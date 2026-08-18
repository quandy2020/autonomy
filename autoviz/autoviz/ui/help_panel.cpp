/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/help_panel.hpp"

#include <QVBoxLayout>

namespace autoviz {

namespace {

QString HelpHtml() {
  return QStringLiteral(
      R"(<h3>Autoviz 帮助</h3>
<p>Autolink 原生 3D 可视化，概念与 RViz2 对齐。</p>

<h4>视图控制器（Views 面板）</h4>
<table cellpadding="4">
<tr><td><b>Orbit</b></td><td>默认轨道透视：左键旋转，中键/Shift+左键平移，滚轮缩放</td></tr>
<tr><td><b>TopDownOrtho</b></td><td>正交俯视：左键平移，滚轮缩放；无旋转</td></tr>
<tr><td><b>FPS</b></td><td>第一人称：WASD 移动，左键环顾；F 切换行走/飞行，R 重置视角</td></tr>
<tr><td><b>FPSMotion</b></td><td>轨道 + 第一人称混合：WASD 移动焦点，左键旋转轨道视角</td></tr>
<tr><td><b>XY Orbit</b></td><td>固定俯仰角，仅水平旋转</td></tr>
<tr><td><b>Top Down</b></td><td>俯视透视</td></tr>
<tr><td><b>Third Person Follow</b></td><td>第三人称跟随</td></tr>
</table>

<h4>相机操作（Interact / Move Camera 工具）</h4>
<ul>
<li><b>Orbit</b> — 左键：Yaw/Pitch；中键或 Shift+左键：平移；滚轮：缩放</li>
<li><b>TopDownOrtho</b> — 左键：平移 X/Y；滚轮：缩放（Distance 属性）</li>
<li><b>FPS / FPSMotion</b> — WASD：移动；左键：环顾（FPS）或轨道旋转（FPSMotion）</li>
</ul>

<h4>工具栏快捷键</h4>
<table cellpadding="4">
<tr><td><b>1</b></td><td>Move Camera</td></tr>
<tr><td><b>2</b></td><td>Select — 左键拾取场景几何；<b>Ctrl+点击</b> 追加多选</td></tr>
<tr><td><b>3</b></td><td>Interact — 拖拽 Interactive Marker；右键菜单；Shift 微调</td></tr>
<tr><td><b>4</b></td><td>Focus Camera — 左键将视点聚焦到地面交点</td></tr>
<tr><td><b>5</b></td><td>Measure — 两次左键测量距离</td></tr>
<tr><td><b>6</b></td><td>2D Nav Goal — 拖拽箭头发布目标位姿</td></tr>
<tr><td><b>7</b></td><td>2D Pose Estimate — 拖拽发布初始位姿</td></tr>
<tr><td><b>8</b></td><td>Publish Point — 左键发布点击点</td></tr>
</table>
<p>Nav Goal / Pose Estimate / Publish Point 的 Topic 可在 <b>Tool Properties</b> 面板配置并写入 <code>.autoviz</code>。</p>

<h4>菜单与全局快捷键</h4>
<table cellpadding="4">
<tr><td><b>Ctrl+O</b></td><td>打开配置</td></tr>
<tr><td><b>Ctrl+Shift+O</b></td><td>打开 Autolink record 并立即播放</td></tr>
<tr><td><b>Ctrl+S</b></td><td>保存配置</td></tr>
<tr><td><b>Ctrl+Shift+S</b></td><td>另存配置</td></tr>
<tr><td><b>Ctrl+Q</b></td><td>退出</td></tr>
<tr><td><b>F11</b></td><td>全屏 / 退出全屏（Panels 菜单）</td></tr>
<tr><td><b>Esc</b></td><td>退出全屏</td></tr>
</table>

<h4>面板</h4>
<p>通过 <b>Panels</b> 菜单可显示/隐藏各面板（与 RViz 一致）；<b>Add New Panel</b> 添加已关闭的面板，<b>Delete Panel</b> 关闭当前可见的面板；菜单末尾列出各面板的 toggle 项。</p>
<ul>
<li><b>Displays</b> — 添加/启用 Display，设置 Fixed Frame</li>
<li><b>Selection</b> — 显示拾取点坐标与所属 Display</li>
<li><b>Views</b> — 保存/应用相机视角，写入 <code>.autoviz</code></li>
<li><b>Time</b> — Time / Wall 时间与 Experimental 同步</li>
<li><b>Playback</b> — 回放 Autolink <code>.record</code>；拖入文件或 File → Open Record 自动打开</li>
<li><b>TF Tree</b> — 变换树</li>
<li><b>Image</b> — 订阅 Image Display 的最新帧</li>
</ul>

<h4>回放与 Bag/MCAP</h4>
<p>与 Foxglove 打开 MCAP 类似：将 Autolink <code>.record</code>（或 <code>.bag</code> / <code>.mcap</code>）拖到窗口上，或使用 <b>File → Open Record...</b>，即可打开并立即播放。也可从命令行传入文件：<code>autoviz path/to/data.record</code>。</p>
<p><code>.bag</code> / <code>.mcap</code> 会先转换成 <code>.record</code>（需要 <code>bag_to_record</code> 或 <code>mcap_to_record.py</code>）。Playback 面板支持播放、暂停、倍速与拖动进度。</p>

<h4>渲染后端</h4>
<p>会话配置 <code>.autoviz</code> 可指定 OpenGL 或 Ogre（需 <code>-DAUTOVIZ_USE_OGRE=ON</code> 编译）。Ogre 使用圆盘点精灵、GGX PBR 着色与 Camera 纹理 quad。</p>

<h4>Display 插件</h4>
<p>设置环境变量 <code>AUTOVIZ_PLUGIN_PATH</code>（<code>:</code> 分隔目录）加载导出 <code>autoviz_register_displays</code> 的 <code>.so</code>。</p>
)");
}

}  // namespace

HelpPanel::HelpPanel(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  auto* browser = new QTextBrowser(this);
  browser->setOpenExternalLinks(true);
  browser->setHtml(HelpHtml());
  layout->addWidget(browser);
}

}  // namespace autoviz
