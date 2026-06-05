#!/bin/bash
# 智能多级目录镜像构建脚本 - 完美支持以 README.md 作为目录首页与文件名控序
set -e

OUTPUT_DIR="${1:-public}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSS_SRC="$SCRIPT_DIR/style.css"
DOC_SRC="$(cd "$SCRIPT_DIR/../doc" && pwd)"

echo "[build-pages] Target Output dir: $OUTPUT_DIR"
echo "[build-pages] Static CSS source: $CSS_SRC"
echo "[build-pages] Markdown sources: $DOC_SRC"

# 1. 初始化根产物目录
mkdir -p "$OUTPUT_DIR/css"

# 2. 部署全局 CSS 样式
if [ -f "$CSS_SRC" ]; then
    cp "$CSS_SRC" "$OUTPUT_DIR/css/style.css"
else
    echo "[ERROR] CSS file missing at $CSS_SRC"
    exit 1
fi

# 3. 编译大项目首页 (根目录 README.md)
if [ -f "README.md" ]; then
    pandoc README.md -f markdown -t html -s \
        --metadata title="visionAlgorithm" \
        -c css/style.css \
        -o "$OUTPUT_DIR/index.html"
else
    echo "<html><head><link rel='stylesheet' href='css/style.css'></head><body><h1>visionAlgorithm</h1></body></html>" > "$OUTPUT_DIR/index.html"
fi

# ========================================================
# 4. 递归扫描、镜像编译并动态收集目录树
# ========================================================
echo "[build-pages] Mirroring directory trees..."
TMP_ENTRIES=".tmp_entries"
rm -f "$TMP_ENTRIES"

if [ -d "$DOC_SRC" ]; then
    # 深度解耦：支持 doc/ 下任意层级的 md 文件检索
    find "$DOC_SRC" -name '*.md' -type f | while read -r md_file; do
        # 计算相对于 doc/ 的相对路径 (例如: common/README.md 或 detector/task1/intro.md)
        rel_path="${md_file#$DOC_SRC/}"
        rel_dir=$(dirname "$rel_path")
        base_name=$(basename "$md_file" .md)
        
        # 提取一级子目录作为大分类标签
        category_dir=$(echo "$rel_dir" | cut -d'/' -f1)
        category=$(echo "$category_dir" | tr '[:lower:]' '[:upper:]')
        
        # 核心规定：只要是 README.md，一律编译为 index.html 作为当前目录首页
        if [ "$base_name" = "README" ]; then
            target_name="index.html"
            sort_key="00_README" # 赋予最高优先级，确保排序在最前面
            display_name="🏠 分类首页 (README)"
        else
            target_name="${base_name}.html"
            sort_key="${base_name}"
            display_name="📄 ${base_name}"
        fi
        
        # 映射并创建镜像输出目录
        if [ "$rel_dir" = "." ]; then
            target_dir="$OUTPUT_DIR/doc"
            rel_path_from_public="doc/$target_name"
        else
            target_dir="$OUTPUT_DIR/doc/$rel_dir"
            rel_path_from_public="doc/$rel_dir/$target_name"
        fi
        mkdir -p "$target_dir"
        
        # 动态数学计算：根据当前编译深度计算返回根目录的相对路径(如 ../../)
        rel_dir_to_public="${target_dir#$OUTPUT_DIR/}"
        to_root=$(echo "$rel_dir_to_public" | sed 's/[^/]\+/../g')/
        
        # 编译生成对应的 HTML 页面
        pandoc "$md_file" -s \
            --metadata title="${base_name}" \
            -c "${to_root}css/style.css" \
            -o "$target_dir/$target_name"
            
        # 收集元数据：分类 | 排序键 | 显示名称 | 相对于 public 根的绝对路径
        echo "${category}:${sort_key}:${display_name}:${rel_path_from_public}" >> "$TMP_ENTRIES"
    done
fi

RAW_ENTRIES=""
if [ -f "$TMP_ENTRIES" ]; then
    # 完美实现：先按大分类聚合，同分类内严格按照文件名(sort_key)顺序显示
    RAW_ENTRIES=$(sort "$TMP_ENTRIES")
    rm -f "$TMP_ENTRIES"
fi

# 5. 修复 Markdown 内部的相对跳转链接（如点击 README.md 自动去往 index.html）
echo "[build-pages] Resolving internal hyperlinks..."
find "$OUTPUT_DIR" -name '*.html' -type f | while read -r html_path; do
    sed -e 's/README\.md/index.html/g' -e 's/\.md/\.html/g' "$html_path" > "${html_path}.tmp" && mv "${html_path}.tmp" "$html_path"
done

# ========================================================
# 6. 为所有镜像 HTML 注入高度路径自适应的折叠侧边栏
# ========================================================
echo "[build-pages] Injecting adaptive collapse sidebars..."
find "$OUTPUT_DIR" -name '*.html' -type f | while read -r html_file; do
    
    # 精准定位当前页面距离根部的阶梯深度
    rel_to_public="${html_file#$OUTPUT_DIR/}"
    dir_part=$(dirname "$rel_to_public")
    if [ "$dir_part" = "." ]; then
        TO_ROOT=""
    else
        TO_ROOT=$(echo "$dir_part" | sed 's/[^/]\+/../g')/
    fi
    
    current_rel_path_from_public="${html_file#$OUTPUT_DIR/}"
    
    # 动态组装高内聚的 HTML 侧边栏
    build_sidebar="<input type=\"checkbox\" id=\"sidebar-toggler\" class=\"sidebar-toggler-input\">
<nav class=\"sidebar\" id=\"sidebar\">
<div class=\"sidebar-header\"><a href=\"${TO_ROOT}index.html\">🏠 visionAlgorithm</a></div>
<ul>"

    # 大项目大首页高亮判定
    if [ "$current_rel_path_from_public" = "index.html" ]; then
        build_sidebar="${build_sidebar}<li><a href=\"${TO_ROOT}index.html\" class=\"active\">📖 项目公告与首页</a></li>"
    else
        build_sidebar="${build_sidebar}<li><a href=\"${TO_ROOT}index.html\">📖 项目公告与首页</a></li>"
    fi

    # 解析动态排序后的目录树数据
    prev_category=""
    while IFS=: read -r cat skey dname rpath; do
        [ -z "$cat" ] || [ -z "$rpath" ] && continue
        
        if [ "$cat" != "$prev_category" ]; then
            if [ -n "$prev_category" ]; then
                build_sidebar="${build_sidebar}</ul></li>"
            fi
            cat_slug=$(echo "$cat" | tr -cd 'A-Za-z0-9')
            
            # checked 代表默认展开侧边栏，去掉 checked 则默认折叠
            build_sidebar="${build_sidebar}
<li class=\"sidebar-group\">
    <input type=\"checkbox\" id=\"cat-${cat_slug}\" class=\"category-toggle\" checked>
    <label for=\"cat-${cat_slug}\" class=\"category-label\"><span>📁 ${cat}</span><span class=\"arrow\">▶</span></label>
    <ul class=\"sidebar-sub-list\">"
            prev_category="$cat"
        fi

        # 精准判定高亮当前活动的子页面
        if [ "$current_rel_path_from_public" = "$rpath" ]; then
            build_sidebar="${build_sidebar}<li><a href=\"${TO_ROOT}${rpath}\" class=\"active\">${dname}</a></li>"
        else
            build_sidebar="${build_sidebar}<li><a href=\"${TO_ROOT}${rpath}\">${dname}</a></li>"
        fi
    done <<< "$RAW_ENTRIES"

    if [ -n "$prev_category" ]; then
        build_sidebar="${build_sidebar}</ul></li>"
    fi

    build_sidebar="${build_sidebar}</ul></nav>
<label for=\"sidebar-toggler\" class=\"sidebar-toggle\">☰</label>
<div class=\"page-content\">"

    # 将拼装完整的树结构灌入 HTML 的 <body> 之后
    awk -v sidebar="$build_sidebar" '
    {
        if ($0 ~ /<body[^>]*>/) {
            print $0
            print sidebar
        } else if ($0 ~ /<\/body>/) {
            print "</div>"
            print $0
        } else {
            print $0
        }
    }
    ' "$html_file" > "${html_file}.tmp" && mv "${html_file}.tmp" "$html_file"
done

echo "[build-pages] Complete! Site compiled in standard structure."
