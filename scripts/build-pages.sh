#!/bin/bash
# ============================================================================
# Docsify 文档站点构建脚本（用于 GitLab CI/CD 或本地构建）
# ============================================================================
# 功能说明：
#   1. 在构建容器中安装 Node.js
#   2. 将 docs/ 目录复制到构建输出目录 public/
#   3. 用 docs/index.html 作为 public/index.html 入口文件
#   4. 自动替换 index.html 中的 CDN（jsdelivr）引用为本地相对路径
#      （提升加载速度，避免 CDN 延迟）
#   5. 生成 public/.nojekyll 文件（GitHub Pages 跳过 Jekyll 处理）
#   6. 复制 scripts/ 下的自定义 CSS 到 public/ 目录
# ============================================================================
# 使用方式：
#   - GitLab CI：CI 流水线自动调用此脚本
#   - 本地构建：bash scripts/build-pages.sh
# ============================================================================

set -e  # 遇到错误立即退出
set -u  # 使用未定义变量时报错

echo "=== 开始构建文档站点 ==="

# -------------------------------------------------------------------
# 第一步：安装 Node.js（Alpine 容器环境）
# -------------------------------------------------------------------
if command -v node &> /dev/null; then
    echo "[INFO] Node.js 已安装，跳过安装步骤"
else
    echo "[INFO] 正在安装 Node.js..."
    apk add --no-cache nodejs npm
    echo "[INFO] Node.js 安装完成: $(node -v)"
fi

# -------------------------------------------------------------------
# 第二步：创建输出目录并复制文档文件
# -------------------------------------------------------------------
echo "[INFO] 创建输出目录 public/"
rm -rf public
mkdir -p public

# 复制 docs/ 下所有文件到 public/
echo "[INFO] 复制 docs/ -> public/"
cp -r docs/* public/

# -------------------------------------------------------------------
# 第三步：替换 index.html 中的 CDN 引用为本地文件
# -------------------------------------------------------------------
# 这一步是可选的：如果你希望保持 CDN 引用请注释掉下面的代码
# 如果你想离线使用请取消注释并确保已下载相关文件

echo "[INFO] 处理 index.html 中的 CDN 引用..."

# 方案 A：将 CDN 替换为本地路径（需要下载对应文件）
# 示例：替换 docsify-themeable JS
# sed -i 's|https://cdn.jsdelivr.net/npm/docsify-themeable@0/dist/js/docsify-themeable.min.js|assets/js/docsify-themeable.min.js|g' public/index.html

# 方案 B：保持 CDN 引用不变（推荐，无需额外维护）
echo "[INFO] 保持 CDN 引用不变"

# -------------------------------------------------------------------
# 第四步：复制自定义 CSS 文件
# -------------------------------------------------------------------
echo "[INFO] 复制自定义 CSS 文件..."
if [ -f scripts/custom.css ]; then
    cp scripts/custom.css public/assets/
fi

# -------------------------------------------------------------------
# 第五步：生成 .nojekyll 文件（GitHub Pages 需要）
# -------------------------------------------------------------------
echo "[INFO] 生成 .nojekyll 文件"
touch public/.nojekyll

echo "=== 构建完成! 输出目录: public/ ==="
ls -la public/