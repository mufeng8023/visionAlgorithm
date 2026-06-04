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
    --metadata lang="zh-CN" \
    -c css/style.css \
    -o "$OUTPUT_DIR/index.html"

# 4. 转换 doc/ 下所有 markdown 文件为 HTML
for md_file in doc/*.md; do
    base_name=$(basename "$md_file" .md)
    pandoc "$md_file" -s \
        --metadata title="${base_name}" \
        --metadata lang="zh-CN" \
        -c ../css/style.css \
        -o "$OUTPUT_DIR/doc/${base_name}.html"
done

# 5. 修复 markdown 内部链接(.md)为 .html
find "$OUTPUT_DIR" -name '*.html' -exec sed -i 's/\(href="[^"]*\)\.md"/\1.html"/g' {} +

# 6. 注入侧边栏导航
# 生成侧边栏顶部(首页链接)
cat > /tmp/sidebar_top.html << 'SIDEBAR_EOF'
<nav class="sidebar" id="sidebar">
<div class="sidebar-header"><a href="../index.html">🏠 首页</a></div>
<ul>
<li class="sidebar-section">📄 详细文档</li>
SIDEBAR_EOF

# 生成每个文档的侧边栏链接
for html_file in "$OUTPUT_DIR"/doc/*.html; do
    name=$(basename "$html_file" .html)
    echo "<li><a href=\"${name}.html\">${name}</a></li>" >> /tmp/sidebar_items.html
done

cat /tmp/sidebar_top.html /tmp/sidebar_items.html > /tmp/sidebar.html
echo '</ul></nav>' >> /tmp/sidebar.html
echo "<div class=\"sidebar-toggle\" onclick=\"document.getElementById('sidebar').classList.toggle('open')\">☰</div>" >> /tmp/sidebar.html

# 对每个 HTML 文件注入侧边栏, 并包装内容
for html_file in "$OUTPUT_DIR"/index.html "$OUTPUT_DIR"/doc/*.html; do
    # 注入侧边栏和 toggle 按钮到 <body> 之后
    sed -i '/^<body>/{
        r /tmp/sidebar.html
        a <div class="page-content">
    }' "$html_file"

    # 在 </body> 之前关闭 .page-content
    sed -i 's|</body>|</div></body>|' "$html_file"
done

# 首页的侧边栏链接路径调整为根目录
sed -i 's|href="\.\./index\.html"|href="index.html"|g' "$OUTPUT_DIR"/index.html
for html_file in "$OUTPUT_DIR"/doc/*.html; do
    sed -i 's|href="\.\./index\.html"|href="../index.html"|g' "$html_file"
done

# 清理临时文件
rm -f /tmp/sidebar_top.html /tmp/sidebar_items.html /tmp/sidebar.html

echo "[build-pages] Done! Output in: $OUTPUT_DIR"