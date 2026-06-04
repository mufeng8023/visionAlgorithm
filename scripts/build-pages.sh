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

# 6. 收集所有 doc 文件名(排除 README, 作为文档首页单独处理)
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
# 通过在构建时将 HTML 结构直接写入, 消除对运行时 JS 生成侧边栏的依赖;
# 这样即使 GitHub Pages 的 CSP 限制内联脚本, 侧边栏依然正常显示;
echo "[build-pages] Injecting sidebar into HTML files..."
for html_file in "$OUTPUT_DIR"/index.html "$OUTPUT_DIR"/doc/*.html; do
    if [ ! -f "$html_file" ]; then
        echo "[WARNING] File not found: $html_file, skipping"
        continue
    fi

    # 判断是首页还是 doc 页面, 决定链接前缀和首页 href;
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

    # 构造侧边栏 HTML 字符串(侧边栏 + 汉堡菜单 + page-content 容器);
    # 注意: 汉堡菜单使用 <input type="checkbox"> + <label> 纯 CSS 实现,
    #       不依赖 JS onclick, 兼容 CSP;
    #       所有链接使用相对路径(构建时已知首页或doc页面), 前后一致;
    build_sidebar="
<input type=\"checkbox\" id=\"sidebar-toggler\" class=\"sidebar-toggler-input\">
<nav class=\"sidebar\" id=\"sidebar\">
<div class=\"sidebar-header\"><a href=\"${home_href}\">🏠 首页</a></div>
<ul>
<li class=\"sidebar-section\">📄 详细文档</li>"

    # 文档首页 README 作为第一条(高亮匹配当前页面);
    if [ "$base_name" = "README" ]; then
        build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}README.html\" class=\"active\">📖 文档首页</a></li>"
    else
        build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}README.html\">📖 文档首页</a></li>"
    fi

    # 其余 doc 链接;
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

    # 使用 awk 一次性完成注入:
    #   1. 在 <body> 之后插入侧边栏 HTML
    #   2. 在 </body> 之前插入 </div> 关闭 page-content 容器
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