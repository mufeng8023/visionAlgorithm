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
// 提取项目根目录基准路径(确保以 / 结尾), 用于构造侧边栏绝对路径链接;
// 避免因 URL 末尾是否带 / 导致的相对路径解析异常;
// 例如: /project-name/doc/BaseNet.html -> /project-name/
//       /project-name/index.html      -> /project-name/
//       /project-name/                -> /project-name/
//       /project-name (无尾随斜杠)     -> /project-name/
//       /index.html                   -> /
//       /                             -> /
var path=window.location.pathname;
var base;
var docIdx=path.indexOf('/doc/');
if(docIdx!==-1){
  // 在 doc 子页面中, 截取到 /doc/ 之前(含尾部 /) 得到项目根路径;
  base=path.substring(0,docIdx+1);
}else{
  // 在首页或非 doc 页面, 判断 pathname 末尾情况以构造正确的根路径;
  var lastChar=path.charAt(path.length-1);
  if(lastChar==='/'){
    // URL 以 / 结尾, 如 /project-name/ , 本身就是根路径;
    base=path;
  }else{
    // URL 不以 / 结尾, 可能是 /project-name 或 /project-name/index.html;
    var lastSlash=path.lastIndexOf('/');
    var lastSeg=path.substring(lastSlash+1);
    if(lastSeg.indexOf('.')!==-1){
      // 末尾是带扩展名的文件, 如 index.html; 去掉文件名得到根路径;
      base=path.substring(0,lastSlash+1);
    }else{
      // 末尾无扩展名且无 /, 如 /project-name; 直接追加 /;
      base=path+'/';
    }
  }
}
// 从 pathname 末尾提取当前文件名(如 "BaseNet.html", "index.html") , 用于高亮匹配;
var lastSlashPos=path.lastIndexOf('/');
var fileName=lastSlashPos>=0?path.substring(lastSlashPos+1):path;
var currentFile=fileName.replace('.html','');
// 使用绝对路径构建侧边栏, 兼容 GitLab/GitHub Pages 各种 URL 格式;
var homeHref=base+'index.html';
var html='<nav class="sidebar" id="sidebar"><div class="sidebar-header"><a href="'+homeHref+'">🏠 首页</a></div><ul><li class="sidebar-section">📄 详细文档</li>';
for(var i=0;i<nList.length;i++){
var nm=nList[i];
html+='<li><a href="'+base+'doc/'+nm+'.html"'+(nm===currentFile?' class="active"':'')+'>'+nm+'</a></li>';
}
html+='</ul></nav><div class="sidebar-toggle" onclick="document.getElementById(\'sidebar\').classList.toggle(\'open\')">☰</div>';
// 将 body 原有内容移入 page-content 容器;
var body=document.body;
var contentWrap=document.createElement('div');contentWrap.className='page-content';
while(body.firstChild){contentWrap.appendChild(body.firstChild);}
body.appendChild(contentWrap);
// 在 body 最前方插入侧边栏;
body.insertAdjacentHTML('afterbegin',html);
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