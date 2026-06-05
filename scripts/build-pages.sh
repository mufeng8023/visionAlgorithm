#!/bin/bash
# 构建 GitLab/GitHub Pages 的统一脚本 - 已适配多级子目录（扁平侧边栏版）
# 依赖: pandoc, sed, awk
# 用法: bash scripts/build-pages.sh

set -e

OUTPUT_DIR="${1:-public}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSS_SRC="$SCRIPT_DIR/style.css"
DOC_SRC="$(cd "$SCRIPT_DIR/../doc" && pwd)"

echo "[build-pages] Output dir: $OUTPUT_DIR"
echo "[build-pages] CSS source: $CSS_SRC"
echo "[build-pages] Doc source: $DOC_SRC"

# 1. 创建输出目录
mkdir -p "$OUTPUT_DIR/css"
mkdir -p "$OUTPUT_DIR/doc"

# 2. 复制 CSS 文件到产物目录
if [ -f "$CSS_SRC" ]; then
    cp "$CSS_SRC" "$OUTPUT_DIR/css/style.css"
else
    echo "[ERROR] CSS file not found at $CSS_SRC"
    exit 1
fi

# 3. 转换根目录 README.md (项目首页) 为首页 index.html
if [ -f "README.md" ]; then
    pandoc README.md -f markdown -t html -s \
        --metadata title="visionAlgorithm" \
        -c css/style.css \
        -o "$OUTPUT_DIR/index.html"
else
    echo "[WARNING] Root README.md not found! Creating a default index.html"
    echo "<html><head><link rel='stylesheet' href='css/style.css'></head><body><h1>visionAlgorithm</h1></body></html>" > "$OUTPUT_DIR/index.html"
fi

# 4. 递归转换 doc/ 及其子目录下所有 markdown 文件为 HTML
echo "[build-pages] Converting markdown to HTML (recursive)..."
if [ -d "$DOC_SRC" ]; then
    # 使用 find 递归查找所有子目录下的 .md 文件
    find "$DOC_SRC" -name '*.md' -type f | while read -r md_file; do
        base_name=$(basename "$md_file" .md)
        
        # 将所有 HTML 统一扁平化输出到 public/doc/ 下，确保 CSS 和相对链接不打破
        pandoc "$md_file" -s \
            --metadata title="${base_name}" \
            -c ../css/style.css \
            -o "$OUTPUT_DIR/doc/${base_name}.html"
    done
else
    echo "[WARNING] doc/ directory not found at $DOC_SRC"
fi

# 5. 修复 markdown 内部链接(.md)为 .html
echo "[build-pages] Fixing internal markdown links..."
find "$OUTPUT_DIR" -name '*.html' -type f | while read -r html_path; do
    sed -i 's/\(href="[^"]*\)\.md"/\1.html"/g' "$html_path"
done

# 6. 收集所有生成的 doc 文件名(排除 README，用来生成扁平侧边栏列表)
echo "[build-pages] Collecting doc filenames..."
DOC_NAMES=""
for html_file in "$OUTPUT_DIR"/doc/*.html; do
    [ -f "$html_file" ] || continue
    name=$(basename "$html_file" .html)
    [ "$name" = "README" ] && continue
    if [ -z "$DOC_NAMES" ]; then
        DOC_NAMES="$name"
    else
        DOC_NAMES="${DOC_NAMES} ${name}"
    fi
done
echo "[build-pages] Doc files found: $DOC_NAMES"

# 7. 遍历所有 HTML 文件, 注入侧边栏静态 HTML 和 page-content 容器
echo "[build-pages] Injecting sidebar into HTML files..."
for html_file in "$OUTPUT_DIR"/index.html "$OUTPUT_DIR"/doc/*.html; do
    if [ ! -f "$html_file" ]; then
        echo "[WARNING] File not found: $html_file, skipping"
        continue
    fi

    # 判断是首页还是 doc 页面, 决定链接前缀和首页 href
    case "$html_file" in
        */index.html)
            link_prefix="doc/"
            home_href="index.html"
            ;;
        */doc/*.html)
            link_prefix=""
            home_href="../index.html"
            ;;
    esac

    base_name=$(basename "$html_file" .html)

    # 构造侧边栏 HTML 字符串
    build_sidebar="
<input type=\"checkbox\" id=\"sidebar-toggler\" class=\"sidebar-toggler-input\">
<nav class=\"sidebar\" id=\"sidebar\">
<div class=\"sidebar-header\"><a href=\"${home_href}\">🏠 首页</a></div>
<ul>
<li class=\"sidebar-section\">📄 详细文档</li>"

    # 文档首页 README 作为第一条
    if [ "$base_name" = "README" ]; then
        build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}README.html\" class=\"active\">📖 文档首页</a></li>"
    else
        build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}README.html\">📖 文档首页</a></li>"
    fi

    # 其余 doc 链接扁平排列
    for dname in $DOC_NAMES; do
        [ -z "$dname" ] && continue
        if [ "$base_name" = "$dname" ]; then
            build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}${dname}.html\" class=\"active\">${dname}</a></li>"
        else
            build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}${dname}.html\">${dname}</a></li>"
        fi
    done

    build_sidebar="${build_sidebar}
</ul>
</nav>
<label for=\"sidebar-toggler\" class=\"sidebar-toggle\">☰</label>
<div class=\"page-content\">
"

    # 使用 awk 将侧边栏注入到生成的 HTML 中
    awk -v sidebar="$build_sidebar" '
    {
        if ($0 ~ /<body[^>]*>/) {
            print
            print sidebar
        } else if ($0 ~ /<\/body>/) {
            print "</div>"
            print
        } else {
            print
        }
    }
    ' "$html_file" > "${html_file}.tmp" && mv "${html_file}.tmp" "$html_file"

    echo "[OK] Injected sidebar into: $html_file"
done

echo "[build-pages] Done! Output in: $OUTPUT_DIR"
