#!/bin/sh
# 构建 GitLab/GitHub Pages 的统一脚本
# 依赖: pandoc, sed
# 用法: sh scripts/build-pages.sh

set -e

OUTPUT_DIR="${1:-public}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSS_SRC="$SCRIPT_DIR/style.css"

echo "[build-pages] Output dir: $OUTPUT_DIR"
echo "[build-pages] CSS source: $CSS_SRC"

# 1. 创建输出目录
mkdir -p "$OUTPUT_DIR/css"
mkdir -p "$OUTPUT_DIR/doc"

# 2. 复制 CSS 文件到产物目录
cp "$CSS_SRC" "$OUTPUT_DIR/css/style.css"

# 3. 转换 README.md 为首页 index.html
pandoc README.md -f markdown -t html -s \
    --metadata title="visionAlgorithm" \
    -c css/style.css \
    -o "$OUTPUT_DIR/index.html"

# 4. 转换 doc/ 下所有 markdown 文件为 HTML
echo "[build-pages] Converting markdown to HTML..."
for md_file in doc/*.md; do
    base_name=$(basename "$md_file" .md)
    pandoc "$md_file" -s \
        --metadata title="${base_name}" \
        -c ../css/style.css \
        -o "$OUTPUT_DIR/doc/${base_name}.html"
done

# 5. 修复 markdown 内部链接(.md)为 .html
find "$OUTPUT_DIR" -name '*.html' -exec sed -i 's/\(href="[^"]*\)\.md"/\1.html"/g' {} +

# 6. 收集所有 doc 文件名
echo "[build-pages] Collecting doc filenames..."
DOC_LIST=""
for html_file in "$OUTPUT_DIR"/doc/*.html; do
    if [ -f "$html_file" ]; then
        name=$(basename "$html_file" .html)
        if [ -z "$DOC_LIST" ]; then
            DOC_LIST="\"${name}\""
        else
            DOC_LIST="${DOC_LIST},\"${name}\""
        fi
    fi
done
echo "[build-pages] Doc files found: $DOC_LIST"

# 7. 生成侧边栏 JS 文件
cat > /tmp/sidebar_inject.js << 'JS_EOF'
<script>
(function(){
var nList=[DOC_LIST];
// 从当前路径判断: 首页(根目录)链接需要加 doc/ 前缀, doc页无需前缀
var isDocIndex=window.location.pathname.indexOf('/doc/')!==-1;
var p2=window.location.pathname.split('/');
var lastPart=p2[p2.length-1];
var isRootIndex=(lastPart===''||lastPart==='index.html');
var prefix='';
if(!isDocIndex&&!isRootIndex){prefix='doc/';}
var ih=isDocIndex?'../index.html':'index.html';
var cF=lastPart.replace('.html','');
var h='<nav class="sidebar" id="sidebar"><div class="sidebar-header"><a href="'+ih+'">🏠 首页</a></div><ul><li class="sidebar-section">📄 详细文档</li>';
for(var i=0;i<nList.length;i++){
var nm=nList[i];
h+='<li><a href="'+prefix+nm+'.html"'+(nm===cF?' class="active"':'')+'>'+nm+'</a></li>';
}
h+='</ul></nav><div class="sidebar-toggle" onclick="document.getElementById(\'sidebar\').classList.toggle(\'open\')">☰</div>';
var b=document.body;
var cw=document.createElement('div');cw.className='page-content';
while(b.firstChild){cw.appendChild(b.firstChild);}
b.appendChild(cw);
b.insertAdjacentHTML('afterbegin',h);
})();
</script>
JS_EOF

# 替换 JS 中的占位符
sed -i "s|DOC_LIST|$DOC_LIST|g" /tmp/sidebar_inject.js

# 8. 在所有 HTML 的 </body> 之前注入侧边栏 JS
# 用 python 或简单的 shell 替换, 兼容所有 sed 版本
echo "[build-pages] Injecting sidebar JS into HTML files..."
for html_file in "$OUTPUT_DIR"/index.html "$OUTPUT_DIR"/doc/*.html; do
    if [ ! -f "$html_file" ]; then
        echo "[WARNING] File not found: $html_file, skipping"
        continue
    fi
    # 方法: 在 </body> 前插入 JS, 使用 awk 兼容所有平台
    awk '{
        if ($0 ~ /<\/body>/) {
            while ((getline line < "/tmp/sidebar_inject.js") > 0) print line
            close("/tmp/sidebar_inject.js")
        }
        print
    }' "$html_file" > "${html_file}.tmp" && mv "${html_file}.tmp" "$html_file"
done

# 清理临时文件
rm -f /tmp/sidebar_inject.js

echo "[build-pages] Done! Output in: $OUTPUT_DIR"