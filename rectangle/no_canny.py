# -*- coding: utf-8 -*-
"""MaixCAM2 靶纸识别 + 靶心定位 + 水平误差计算。

题目：激光光斑距中央竖线 <= 3cm，**只补偿水平误差，竖直方向不做控制**。

算法来源
--------------------------------------------------------------------------

* 父子轮廓面积比筛选 / 极角排序 / 固定尺寸逆透视 / 闭运算
    取自 江南大学《视觉代码带注释.py》(GPL-3.0)
    —— 抗干扰与毫米精度的来源。

关键设计
--------------------------------------------------------------------------
1. **锁曝光 + 固定阈值**，不用自适应阈值。
   相机默认自动曝光，会盯着整张图的平均亮度不停改曝光，靶框的灰度就跟着漂
   -> 阈值判断时对时错 -> 屏幕上的 area 乱跳。锁住之后灰度变成常量，固定阈值
   才能稳稳坐在"黑框"和"背景"之间的那段空档里。

   走过弯路：最初用的是 cv2.adaptiveThreshold（只问"比周围 31x31 的平均值暗吗"），
   但**黑框的粗细随距离变化近 10 倍** —— 拉远时邻域跨到框两侧的背景、平均值被
   稀释；拉近时邻域全落在框内部、把框从里面掏空。扫遍 (C, block) 全组合，没有
   一组能覆盖全部工作距离，最好也只覆盖约 2.5 倍。
   固定阈值是**尺度无关**的（"够不够黑"是绝对判断，与框粗细无关），代价是怕
   亮度变化 —— 所以必须配锁曝光。两者是一套，拆开哪个都不成立。

2. 主循环里**不做 warpPerspective**。靶心和激光点都只是 3x3 矩阵乘一个点：
       center_orig = M_inv @ (RECT_W/2, RECT_H/2)
       laser_corr  = M     @ (LASER_X_PX, LASER_Y_PX)
   不需要任何像素输出，省 3~4ms/帧，并消掉"校正图二次轮廓检测"这个失效模式。
   这在数学上严格等价（射影变换保直线，矩形中心的投影即对角线交点）。
   校正图仅在 DEBUG_THUMBNAIL 打开时才 warp，用于肉眼确认逆透视方向。

3. 闭运算不可省。锁曝光 + 固定阈值解决的是**环境光**，不管**激光反光** —— 激光
   打在黑边上烧出的亮斑会把边框打断，仍需 morphologyEx(MORPH_CLOSE) 跨越。

符号约定
--------------------------------------------------------------------------
    err_x = 靶心 - 激光点        正值 = 激光点在靶心左侧，需右移

err_x_px（原图像素，串口控制用）与 err_x_mm（靶面毫米，判 3cm 用）**同号**。
控制量发像素而非毫米：相机与激光刚性固连、一同随云台转动，角度误差 θ 对应
图像位移约 f*θ，**与距离无关**，环路增益恒定；发毫米的话同样 1° 误差在 1.5m
处对应的毫米数是 0.5m 处的 3 倍，远了会变冲。
若云台转向装反，把 ERR_X_SIGN 改成 -1。

标定步骤（现场按顺序做）
--------------------------------------------------------------------------
0. 【第一步，只做这一件】锁曝光：开机看控制台那两行"设定 -> 读回"是否一致。
   然后**靶纸摆着别动**，盯屏幕上的 area —— 该稳的时候稳不住就是漂移。
   晃几个百分点 = 锁住了；大范围乱跳 = 没锁住，后面全部白调。
1. FIXED_THRESHOLD（配置区里那个值）：**点右上角 BIN 切到二值图**，直接看黑框连不连续。
   断口多 = T 偏低（框没进来）；背景大片变白 = T 偏高。
   定量办法是触摸打点：点一下黑框、再点一下背景，读屏幕上那行 "(x,y) gray NN"，
   T 取在两者中间，但**必须比黑框灰度高出 20~30**，否则框会被拦腰切断。
   ⚠️ **换了场地必须重新打一次点**，别沿用旧数。原因不是"曝光漂了"——
   锁曝光锁住的是"相机怎么响应光"，锁不住"现场有多少光照到胶带上"：
   换场地、换灯、拉个窗帘，胶带灰度都会变。而且 EXPOSURE_US = 0 时，
   新场地开机会**重新检测光照、锁一个不同的曝光值**，连相机响应也跟着变。
   实测同一张照片里胶带灰度能从 39（背光那一边）到 79（被灯照到那一边）；
   这也是 T 必须取在胶带最亮处之上的原因。
   （盯 area 也有用，但别把它当绝对值看 —— 拉远变小、拉近变大都正常。要抓的是
   突然跳 / 变成 0 / 明显比绿框圈住的范围大这三种异常。）
2. CLOSE_KERNEL_SIZE 5：等 T 定住后再动它。作用是把激光亮斑烧出的断口跨接起来。
   开太大 -> 断口是跨过去了，但黑框的内孔也会被一起糊掉。
3. MAX_FRAME_FILL 0.50：判"是框不是块"的阈值，实心块接近 1、中空的框很小
4. MIN_AREA：把靶纸放到最远工作距离，读此时外框像素面积，取其一半左右
5. LASER_X_PX：靶纸放好让光斑落靶，读光斑像素 x
6. 验证毫米换算：靶纸水平平移 50mm，确认 err_x_mm 变化约 50mm
"""

import os
import time as pytime
from struct import pack

import cv2
import numpy as np

try:  # 离线自测时不导入 maix
    from maix import app, camera, display, image, nn, pinmap, touchscreen, uart
    from maix import time as maix_time

    MAIX_AVAILABLE = True
except ImportError:  # 在 PC 上跑 python no_canny.py <图片>
    MAIX_AVAILABLE = False


# =============================================================================
# §3 配置区 —— 现场调参只改这一段
# =============================================================================

# --- 采集 ---
# 2026-09-14 从 512x320 降到 416x260，为了帧率。设备实测：
#     512x320 -> detect 25.8ms -> 36fps     (实测)
#     416x260 -> detect ~18.5ms -> ~53fps   (按像素数推算, 现用档)
# 依据：detect 里 77% 花在 preprocess + cv2.findContours 上，这两个都是
# "逐像素扫全图"，耗时只跟像素数成正比，跟画面内容无关。所以降像素是唯一
# 有效的大杠杆（砍绘制只能省 1.8ms）。
#
# ⚠️ 换分辨率 = 换尺度，下面所有【以像素为单位的门槛】都要按比例重标，
#    否则它们的含义就变了。换算：
#         线性比例 416/512 = 0.8125
#         面积比例 0.8125^2 = 0.66
#    按这两个数改过的【只有】：
#        MIN_AREA       1350 -> 890    (×0.66)
#        MIN_PERIMETER    60 -> 49     (×0.8125)
#        MIN_SIDE_PX      20 -> 16     (×0.8125)
#
#    ⚠️ 有两类**不要**跟着缩：
#    ① 纯比例和角度类的判据（长宽比、内外比、填充占比、容忍角）—— 无量纲，不用动
#    ② CLOSE_KERNEL_SIZE / RING_GROW_SIZE —— 它们是**容差**，不是物体尺寸门槛。
#       实测教训：按 0.8125 把 RING_GROW 从 7 缩到 6，杂乱那张照片的 ring 从
#       0.826 掉到 0.724，直接被闸7(MIN_RING_OVERLAP=0.80)拒掉、整张漏检。
#       图变小之后噪声占比反而更大，容差只能放宽不能收紧。所以这两个保持原值。
CAM_W = 416
CAM_H = 260

# --- 曝光锁定（第一步实验）---
# 相机默认自动曝光，会不停根据"整张图平均亮度"调整，导致靶框的灰度随
# 目标远近漂移 -> 阈值判断时对时错 -> 屏幕上的 area 乱跳。
# 锁曝光就是把相机的手动挡固定住，让灰度只取决于现场灯光。
# 当前设成 True：**上电让自动曝光收敛，然后把曝光和增益一起钉住**。
#
# 理由：灰度阈值要成立，前提是"灰度在一次运行内是常量"。持续自动曝光做不到
# —— 靶纸拉近拉远、激光一亮，相机就重新调，胶带灰度跟着动，阈值就失去意义。
# 所以必须锁。
#
# ⚠️ 但"写一句 cam.exposure(值)"不等于锁上了。要真的锁住需要三件事，都在
# init_camera() 里，缺一条就退化成【持续自动曝光】：
#   ① 收敛时把【曝光】和【增益】两个值都读出来（手动模式下增益是独立的一套）
#   ② 切到手动曝光模式（不切的话 ISP 的自动曝光会一直覆盖我们设的值）
#   ③ 设完读回验证 —— 设定值和读回值不一致，就是没锁上
LOCK_EXPOSURE = True
# 曝光值（微秒）。
#   0  = **上电后先测现场光照**：让 ISP 的自动曝光自己收敛，把它选出的值读出来
#        钉住。这样换场地/换灯光不用手改数值 —— 推荐用这个。
#   >0 = 跳过检测，直接锁到这个数（想复现某次的固定表现时用）。
#
# 标定目标：控制台那行"画面平均亮度"落在 100~200 就算健康。
#
# ⚠️ 已知事实（2026-09-13 实测）：**不切手动曝光模式时，钉住的这个值可能并不
# 真正生效**（表现：填 10 和填 10000，平均亮度都是 255；画面看起来正常是因为
# ISP 自己在曝光）。试过 `cam.exp_mode(camera.AeMode.Manual)` 切手动 —— 值确实
# 生效了，但手动模式下增益是另一个值，同样 10000µs 直接过曝。
# 所以"上电检测再钉住"这条路的实际效果，要看上板后的表现。
EXPOSURE_US = 5000
GAIN = 1        # 填一个【小正数】就行（1、5、100 都一样）。别用 -1。
                       #
                       # 实测（2026-09-14）：cam.gain(v) 的参数只要 > 0，设进去之后
                       # 读回都是同一个数 4376 —— 填 1 还是填 100 都一样。
                       # 也就是说这个参数只是"打开手动增益"的开关，**具体增益由硬件
                       # 定死**，我们改不了。
                       #
                       # ⚠️ 千万不要填 -1（那会拿"收敛时读到的那个增益"去设）——
                       # 自动曝光经常收敛到 7257 这种大数，那个值设进去会被改成
                       # 31182（4.3 倍），画面直接过曝、黑框完全识别不到。
                       # 小正数走的是另一条路，固定在 4376，稳。

# --- 预处理 ---
GAUSSIAN_BLUR_SIZE = 5          # 高斯核，奇数
ADAPTIVE_BLOCK_SIZE = 31        # 自适应阈值邻域边长，必须为奇数
# 阈值偏移：要比周围暗多少才算黑。增大 -> 更严格 -> 前景像素变少。
# C=80 是实测出来的（起始值 18，现场又抬上去）：杂乱背景（瓷砖地/垫子/反光）
# 下 C=8 会让背景整片变成前景并与靶框连成一体，四边形被撑到整张图（bbox 近整幅
# 416x260）、靶心乱跳；提到 18 后偏离从 42.8px 降到 8.1px。
# ⚠️ 那套「安全区间 14~22 / 超过 26 漏检」是当时(C=18 阶段)的结论，**不是** 80 的依据。
# 它是【休眠常量】：读取点只有两处 —— preprocess() 里 ADAPTIVE_BLOCK_SIZE 旁边的自适应
# 分支（THRESH_MODE="fixed" 时走不到），以及 draw_touch_info() 触摸打点浮层（缓存键 +
# 非 fixed 时的判定，而那个浮层只画点不参与识别）。所以照现在的配置，它的取值不影响
# 任何识别行为，改改也不会更准；只有把 THRESH_MODE 换成 "adaptive" 它才真的起作用。
ADAPTIVE_C = 80
# 闭运算核；0 或 1 表示关闭。作用是把阈值漏掉的断口跨接起来。
# 实测 5 比 15 好：核太大反而会把东西糊到一起（杂乱背景那张照片上，
# 15 的中心偏离 47.6px，5 只有 9.3px）。
# 激光光斑造成的断口如果 5 跨不过去，再往上加，但每次加完都要重新确认中心准不准。
CLOSE_KERNEL_SIZE = 5

# --- Canny 边缘通道 ---
# 与固定阈值互补的第二路：阈值问"够黑吗"（看绝对值，反光会让它失效），
# Canny 问"这里灰度跳变吗"（看梯度，边框被照亮了它照样画得出线）。
# 两路 OR 起来，阈值漏掉的断口由边缘补上。缺了这一路，固定阈值就会把黑框切碎。
# 代价：Canny 也会把背景的矩形（窗户、门框、白板）描出来，形状和靶框一样、
# 还可能更大，而选框规则是"取最大"，于是它们能顶掉靶纸（表现为画面跳变）。
# 这道副作用由下面的 MIN_RING_OVERLAP 挡。
# 想单独验证"断口到底该谁补"，可以临时改成 False 对比：关掉后黑框的断口会露出来。
USE_CANNY = False
CANNY_LOW = 80
CANNY_HIGH = 150

# 轮廓"压在暗区上"的最低比例，用来挡掉 Canny 描出来的背景矩形。
#框趋近1
MIN_RING_OVERLAP = 0.95
# 上面那个判据比对前，先把 region 往外膨胀这么多像素（7 = 往外放 3 像素）。
# 理由是阈值只抓得住黑框的一部分（反光、灯光不均处会漏），膨胀给它兜底。
# 只影响这一个判据，且每帧只做一次。
RING_GROW_SIZE = 7


# --- 二值化方式 ---
# "fixed"    固定阈值：只问"这个像素绝对够黑吗"。中灰背景进不来，不会误识别背景；
#            代价是它"保守"，靠 Canny/闭运算补漏（第二、三步再说）。曝光锁住后首选。
# "otsu"     每帧自动算一个全局阈值。能跟着场地灯光走，但画面里暗东西一多就会漂。
# "adaptive" 自适应阈值：只问"比周围暗吗"。光照免疫，但背景纹理会被大量收进来，
#            实测会误识别背景矩形 —— 这是被换掉的原因。留在这里只作对比用。
THRESH_MODE = "fixed"
# 固定阈值。黑框要比它暗、其他东西要比它亮。
# 关键：**它必须落在"黑框"和"其它东西"之间那段灰度空档里**，太低会切进黑框
# 自身（边框被打断）、太高会放进背景。
#
# 取值办法：触摸打点读两个数 —— 点黑框、点背景，阈值取在两者中间，
# 但**必须明显高于黑框的灰度**，否则框会被拦腰切断。留 20~30 的余量。
#
# ⚠️ 为什么从 78 抬到 110（2026-09-13 实拍证据）：
# 靶面是 A4 + 1.8cm 黑胶带，胶带**同一张照片里**灰度就能从 39（背光的下边）
# 一直变到 79（被灯照到的上边）。沿靶心竖着扫那张实拍图：
#     y= 50 gray= 79    ← 上边胶带
#     y= 60 gray= 78    ← 正好等于原来的阈值
#     y= 70 gray=202    ← 白面板
# 而 THRESH_BINARY_INV 判的是 "gray < T"，78 < 78 为假 —— **上边框那一整条
# 根本没进阈值通道**，全靠 Canny 那 1 像素的线吊着。那条线一断，轮廓形状就变、
# 另一个候选顶上来，表现就是"矩形跳变 / 类矩形误判"。
# 抬到 110 之后：胶带 32~79 全进得来（余量 31 起），白面板 187~203 仍在背景侧
# （余量 77 起），两边都宽敞。
# 实测：T 从 55 扫到 110，三张真实照片的靶心坐标**一个像素都不变**，130 才开始
# 变化 —— 所以抬上去没有副作用，可选区间很宽。取值依据是"离黑框灰度有多远"，
# 不是"哪个数最好"。
# 现场仍建议打点复核一次：换场地/换灯光后胶带灰度会变，报出来的数要重新对。
#
# ⚠️ 不要为了"让框闭合"去抬 T —— 抬 T 会让背景大量涌进来（这是反二值化，
# 比 T 暗的算前景，T 越大进来的越多）。框断口应该交给 Canny 和闭运算补。
FIXED_THRESHOLD = 100

# --- 初筛 ---
# 最小轮廓面积(px^2)，416x260 下。
#
# ⚠️ 这个数是【唯一一个跑在其它所有判据之前】的关卡，所以它同时决定两件事：
#    ① 帧率 —— 它下面那 8 道闸，每调用一次要花约 10us 的固定开销（cv2 调用本身
#       的开销，跟候选多大无关）。设备实测"各道闸"最高一次 7.5ms ≈ 750 次调用。
#       在门口挡掉一个碎块，就省下后面 8 次调用。
#    ② 误检 —— 之前出现"锁到 area=1200 的小碎块"，根因就是这里太低。
#
# 实测真靶纸在 416x260 下的面积：12546 / 43110 / 57790（三张照片）。
# 取 1500：比最小的真靶纸还低 8 倍（远距离绝不会漏），但已经能挡掉
# "area≈1200 的小碎块被误识别"那类东西，也比原来的 890 高 1.7 倍。
#
# 这个值偏保守 —— 真正该做的是按本文件"标定步骤"第 4 条标定：
# 靶纸放到最远工作距离，读那时的 area，取其一半。
MIN_AREA = 1300
MIN_PERIMETER = 49              # 最小轮廓周长(px)（60 × 0.8125）

# 四边形贴合度下限：轮廓面积 / 它的最小外接矩形面积。
#   完整矩形 = 1.00 ；圆 ≈ 0.785 ；歪扭的碎块更低。
#
# 这一道【插在 approximate_quad 之前】，是纯为了帧率加的 —— 但它同时也是
# 一个货真价实的形状判据：
#     贴合度： contourArea + minAreaRect        ← 很便宜
#     approximate_quad：凸包 + 7 次 approxPolyDP ← 贵几十倍
#   碎块在这里就被扔掉，后面的凸包、approxPolyDP、以及那 8 道闸（设备实测
#   每个候选约 100us）全都省了。设备上"各道闸"最高一次 9.2ms，就是这么来的。
#
# ⚠️ 必须用 minAreaRect（带旋转），不能用轴对齐的外接矩形：
#     轴对齐的话，矩形转 45° 时这个值会掉到 0.5，会把转着角度的靶纸自己杀掉。
#     实测 minAreaRect 版本在 0~60° 旋转下都稳定在 0.987~1.000。
#
# 实测取值（三张照片，416x260）：
#     最终被锁定的候选   0.931 / 0.944 / 0.979
#     其它碎块          0.748 ~ 0.797
# 取 0.85：碎块全挡掉，被锁定的那个最小也有 0.931，余量 0.08。
# ⚠️ 只在三张照片上验证过 —— 现场若发现真靶纸漏检，先把这里降到 0.80 试试。
MIN_QUAD_FIT = 0.85

# --- 四边形近似 ---
# 斜视时轮廓点疏密不均，单一 epsilon 会在近处过粗、远处过细，故多组试探
APPROX_EPSILON_CANDIDATES = (0.010, 0.015, 0.020, 0.025, 0.030, 0.035, 0.040)
APPROX_AREA_RATIO_LO = 0.80     # 近似后面积/原面积 的允许区间
APPROX_AREA_RATIO_HI = 1.20

# --- 几何校验 ---
# 这两个数是"像不像矩形"的主要判据，也是挡"类矩形误判"的第一道闸。
# 收紧到 20 度 / 0.20 的依据是三张真实靶纸照片的实测值：
#   四角偏离 90 度的最大值  9.2 度  -> 留 2.2 倍余量
#   对边长度差占比最大值    0.082   -> 留 2.4 倍余量
# （原先取 30 度 / 0.40，比实测松了 3~5 倍，等于没在筛）
# ⚠️ 别收得比这更狠：测试只覆盖了三张照片，极端斜视的角度没测到。
# 如果现场发现真靶纸被漏检，先把这两个数各自放宽 5 度 / 0.05 试试。
ANGLE_TOLERANCE_DEG = 20     # 四角与 90 度的允许偏差
SIDE_RATIO_TOLERANCE = 0.20     # 对边长度差 / 较长边
MIN_SIDE_PX = 16                # 最短边，排除过细线段（20 × 0.8125）
MAX_SIDE_ASPECT = 10.0          # 最长边/最短边 的上限（照搬江南，挡细长条）
# 长宽比：靶纸是 A4 横放（297:210 = 1.414），始终是"横的"。
# 斜视会让表观长宽比按 cos(偏角) 压缩，所以下限不能卡到 1.414：
#   45 度偏角 -> 1.414*0.707 = 1.00 ; 60 度 -> 0.71
# 取 0.75 可容忍约 58 度偏角，同时挡掉竖长条干扰（窗框、门框、显示器竖屏）。
MIN_ASPECT = 0.75
MAX_ASPECT = 2.80

# --- "是框不是块"判据（抗干扰核心）---
# 取外轮廓填充后，其内部前景像素的占比。实心块接近 1，中空的框很小：
#   占比 = 1 - (W-2t)(H-2t)/(W*H)，t 为黑边宽度
#   A4 横放配 18mm 边 -> 0.27 ; 25mm -> 0.37 ; 50mm -> 0.65
# 取 0.65 可容纳到约 50mm 的黑边，同时把实心暗块（黑裤子、桌沿）挡在外面。
#
# 注：江南代码用的是"父子轮廓面积比 >= 0.7"，但那个判据依赖固定阈值——
# 靶纸内部的同心圆在自适应阈值下会变成前景，闭合圆环把内孔切成窄环带，
# 面积比失效。改用填充占比后与内孔是否被切碎无关。
MAX_FRAME_FILL = 0.30

# 内孔面积 / 外框面积 的下限 —— 用内外两个轮廓互相印证"这是个规整的框"。
# 靶纸的框粗细是固定的，所以这个比值应该是个稳定值；比值掉下来只有一个原因：
# **外框被背景污染撑大了**（黑框和三脚架/背景糊在一起），而这正是"四边形被撑歪"
# 的根源。所以它比角度判据更能抓"外框粘连"。
#
# ⚠️ 为什么从 0.75 降到 0.68（2026-09-13 按官方规格查证）：
# 真靶纸的这个比值是**物理固定**的，可以直接算出来 ——
# 2025 电赛 E 题：靶面 A4 幅面，1.8cm 宽黑胶带沿四周边缘贴：
#     外沿 = A4 边           297 x 210        = 62370 mm^2
#     内孔 = 去掉两圈胶带     (297-36)(210-36) = 261 x 174 = 45414 mm^2
#     比值 = 45414 / 62370 = 0.728
# 而阈值是 0.75 —— **比真值还高 0.022**。也就是说这道闸只要有机会发作，
# 它拒掉的一定是干净的真靶纸，而不是被污染的外框。方向是反的。
# （原先注释引用的"干净时 0.869"是量自一张边框只有约 8mm 宽的合成图，
#   不是 1.8cm 胶带 —— 标定样本和实物对不上，所以定出了这个反的值。）
#
# 取 0.68：干净 0.728 > 0.68 > 污染 0.642，两边都留出余量，这才接回它注释里
# 说的那个判据。换胶带宽度可以按同一个式子反算，**阈值必须取在实际 t 算出的
# 值之下**，否则会误杀真靶纸：
#     ratio = (297-2t)(210-2t) / (297*210)
#     t=15mm -> 0.771   t=18mm -> 0.728   t=20mm -> 0.700   t=25mm -> 0.634
#
# 注意它只在"外框确实有内孔"时才能用（框断掉时外轮廓没有子轮廓，此时跳过检查）。
# 0 表示关闭。
MIN_INNER_OUTER_RATIO = 0.68

# 内孔面积 / 外框面积 的**上限** —— 挡"框太细"的干扰：显示器边框、画框、打印的
# 细线框。这类东西的比值接近 1，下限完全挡不住。同一个式子反算（框宽 t）：
#     t=18mm(真靶纸) 0.728   t=12mm 0.814   t=10mm 0.844   t=5mm 0.920
# 取 0.85 ⇒ 要求框宽 >= 约 10mm，比真靶纸的 18mm 让出 8mm；真值 0.728 到上限
# 还有 0.122 间隙，够扛边缘模糊/透视/固定阈值带来的面积抖动（约 ±0.03~0.05）。
# ⚠️ 和下限一样，只在"确实有内孔"(child_area >= MIN_AREA) 时才生效。0 表示关闭。
MAX_INNER_OUTER_RATIO = 0.82

# --- 靶纸物理尺寸与校正空间 ---
# ⚠️ 这里填的必须是**算法实际锁定的那个四边形**的物理宽度，不是想当然的靶纸外沿。
# 实测：黑框与上方数字牌、下方三脚架在二值图里连成一体，这个合并体的外轮廓不是
# 四边形、过不了几何校验；真正被锁定的是黑框的**内孔**（即白色面板本身）。
# 所以当框比面板宽出一圈时，TARGET_WIDTH_MM 应填**面板**的宽度而不是外框宽度，
# 否则毫米换算会整体偏大（框越厚偏得越多）。
#
# err_x_px 不受影响（它是像素，与尺度无关，串口控制用的是它）。
# 只有 err_x_mm（判 3cm 用的显示值）会被这个常量影响。
#
# 最省事的确认办法就是标定第 6 步：靶纸水平平移已知距离，
# 看 err_x_mm 变化对不对得上，不对就按比例改这个数。
TARGET_WIDTH_MM = 261.0
TARGET_HEIGHT_MM = 174.0
RECT_PX_PER_MM = 1.5
RECT_W = int(TARGET_WIDTH_MM * RECT_PX_PER_MM)      # 445
RECT_H = int(TARGET_HEIGHT_MM * RECT_PX_PER_MM)     # 315
MM_PER_PX_RECT = TARGET_WIDTH_MM / RECT_W           # 约 0.667 mm/px

# --- 激光点（物理标定常量）---
# err_x = 靶心 - 激光点。两者同号。
LASER_X_PX = CAM_W / 2.0        # 默认取画面中心；标定后改成实测像素 x
LASER_Y_PX = CAM_H / 2.0        # 仅用于绘制，竖直方向不参与控制
ERR_X_SIGN = -1                  # 云台转向装反时改成 -1

# --- 串口（沿用原有 6 字节协议，未改动）---
UART_DEVICE = "/dev/ttyS4"
UART_TX_PIN = "A21"
UART_RX_PIN = "A22"
UART_BAUDRATE = 115200

# AA 55 STATUS ERR_X_LO ERR_X_HI CHECKSUM，校验为前 5 字节之和
FRAME_HEADER = b"\xAA\x55"
STATUS_TARGET_VALID = 1
STATUS_NO_TARGET = 0

# --- 调试 ---
# 耗时详情总开关。
#   False（当前）：屏幕上只留一行 "fps XX.X"，控制台不打 [PROFILE]
#   True         ：屏幕上多两行各阶段耗时，控制台每 150 帧打两行
# 不管开关怎么设，屏幕上那行帧率【一直都在】—— 转云台时要知道够不够快。
# 计时本身也一直在跑（每帧就几次 perf_counter，量不出来），改回 True 立刻能看。
PROFILE = False

# 屏幕上那个 fps 用最近多少轮的平均值。单轮的周期会抖（取流、检测、送屏各自
# 都在变），直接显示单次采样的话数字会来回跳。取 15 轮 ≈ 三分之一秒。
FPS_AVG_N = 15
PROFILE_EVERY = 15              # 只用来定控制台那份的间隔（每 15*10=150 帧打一行）。
                                # 屏幕上的那份是每帧都画 —— 不每帧画会一闪一闪。
# 是否画绿框/靶心/err_x 那一层叠加。
# 关掉（False）可以量出"屏幕绘制到底占多少帧时间" —— 对比开/关两种情况下的 fps 就知道。
# ⚠️ 关掉之后屏幕上只有原始画面，看不到检测结果，量完记得改回 True。
# 用这个开关，**不要去注释代码** —— 上次注释多了一行，把 show_img = img 也注掉，
# 板子直接 UnboundLocalError 崩了。
DRAW_RESULT = True
DEBUG_TOUCH = False             # 触摸打点显示该点灰度与二值化判定
DEBUG_BIN_BUTTON = False         # 右上角放一个按钮：点一下在"原图/二值图"之间切换
BIN_BTN_W = 84                  # 按钮尺寸
BIN_BTN_H = 26
DEBUG_THUMBNAIL = False         # 打开后才 warp 校正图贴到屏幕角落
THUMBNAIL_W = 150               # 缩略图宽度(px)
THUMBNAIL_EVERY = 10            # 每 N 帧更新一次缩略图（warp 约 3ms，别每帧做）


# =============================================================================
# §4 串口（协议原样保留）
# =============================================================================


def build_uart_frame(status, err_x):
    """组装 6 字节帧：AA 55 STATUS ERR_X(int16 LE) CHECKSUM。"""
    status = int(status) & 0xFF
    err_x = max(-32768, min(32767, int(round(err_x))))
    frame = FRAME_HEADER + pack("<Bh", status, err_x)
    return frame + bytes((sum(frame) & 0xFF,))


def init_uart():
    """初始化 UART；失败时返回 None，主循环会降级为无串口运行。"""
    if not MAIX_AVAILABLE:
        return None

    # 先列出可用串口，开不起来时这行就是排查依据
    try:
        print("可用串口:", uart.list_devices())
    except Exception:
        pass

    try:
        pinmap.set_pin_function(UART_TX_PIN, "UART4_TX")
        pinmap.set_pin_function(UART_RX_PIN, "UART4_RX")

        # 只传设备名和波特率 —— 默认就是 8 位数据 / 无校验 / 1 位停止位，
        # 也就是我们需要的 8N1。
        # 不要显式传 uart.BITS_8 之类的常量：不同 MaixPy 版本里这些常量的
        # 名字不一样（有 uart.BITS.BITS_8，也有 uart.BITS_8），传错的话整个
        # 串口会直接开不起来（实测报 "has no attribute 'BITS_8'"）。
        return uart.UART(UART_DEVICE, UART_BAUDRATE)
    except Exception as exc:
        print("UART 初始化失败，降级为无串口运行:", exc)
        return None


# =============================================================================
# §4.5 数字识别阶段 —— 上电先跑一次，跑完切回矩形识别
# ---------------------------------------------------------------------------
# 数字模型要 416x416 正方形输入，而矩形识别那套阈值全按 416x260 标定的，
# 换分辨率就全部作废。所以这一段把相机切到 416x416，跑完再切回来。
# =============================================================================

DIGIT_MODEL_PATH_LOCAL = "model_320576.mud"
DIGIT_MODEL_PATH_SYS = "/root/models/maixhub/320576/model_320576.mud"

DIGIT_CAM_W = 416
DIGIT_CAM_H = 416

DIGIT_CONF_TH = 0.4         # MaixHub 示例用 0.5，放宽；误检交给连续帧确认去滤
DIGIT_IOU_TH = 0.45

# 连续多少帧都是同一个数字才认。单帧准确率 93.2%（模型报告里的 val_acc），
# 连 5 帧能把误判压到千分之几。
DIGIT_CONFIRM_N = 5

# 认不出来就跑这么久然后放弃（给底盘发 0，让它知道没认出来）。
# ⚠️ 不含下面 DIGIT_REPEAT_SECONDS 那一段。
DIGIT_MAX_SECONDS = 10.0

# 确认后往底盘重复发多久。**不需要反向 ACK** —— 发几十遍总有一遍能到，
# 两边都只做单向，链路最少。
DIGIT_REPEAT_SECONDS = 3.0
DIGIT_SEND_HZ = 20.0

# --- 第二路串口（发给底盘）---
# 引脚按 MaixCAM2 Pins v1.0 引脚图：A30 = UART1_TX, A31 = UART1_RX
# 设备名规律：UARTn -> /dev/ttySn（云台那路是 UART4 -> /dev/ttyS4）
CHASSIS_UART_DEVICE = "/dev/ttyS1"
CHASSIS_UART_TX_PIN = "A30"
CHASSIS_UART_RX_PIN = "A31"
CHASSIS_UART_BAUDRATE = 115200

DIGIT_FRAME_HEADER = b"\xAA\x55"
DIGIT_FRAME_CMD = 0x4E                  # 'N'


def build_digit_frame(digit):
    """5 字节帧：AA 55 'N' <digit> <checksum>，checksum = 前 4 字节之和 & 0xFF。

    ⚠️ 这个格式要跟底盘那边对齐 —— 把这段发给负责底盘的人。
        digit = 1~4 是识别到的数字；0 表示没认出来（超时）。
    """
    body = DIGIT_FRAME_HEADER + bytes((DIGIT_FRAME_CMD, int(digit) & 0xFF))
    return body + bytes((sum(body) & 0xFF,))


def init_uart_chassis():
    """发给底盘的那路串口。失败返回 None，数字发不出去（不致命）。"""
    if not MAIX_AVAILABLE:
        return None
    try:
        pinmap.set_pin_function(CHASSIS_UART_TX_PIN, "UART1_TX")
        pinmap.set_pin_function(CHASSIS_UART_RX_PIN, "UART1_RX")
        return uart.UART(CHASSIS_UART_DEVICE, CHASSIS_UART_BAUDRATE)
    except Exception as exc:
        print("底盘串口初始化失败（数字发不出去）:", exc)
        return None


def _send_digit(dev, digit):
    """往底盘重复发数字，持续 DIGIT_REPEAT_SECONDS 秒。"""
    if dev is None:
        print("底盘串口不可用，数字没发出去:", digit)
        return
    frame = build_digit_frame(digit)
    n = max(1, int(DIGIT_REPEAT_SECONDS * DIGIT_SEND_HZ))
    period = 1.0 / DIGIT_SEND_HZ
    print("向底盘发送 N=%d，共 %d 次" % (digit, n))
    for _ in range(n):
        try:
            dev.write(frame)
        except Exception as exc:
            print("底盘串口发送失败:", exc)
            return
        pytime.sleep(period)


def _set_resolution(cam, w, h):
    """切相机分辨率，并丢掉切换后 ISP 还没稳的那几帧。

    帧率模式是创建 Camera 对象时按 (w, h, fps) 定的，两个阶段的分辨率都属于
    同一档，所以创建后 set_resolution 够用 —— 若实测切完帧率不对，就得改成
    销毁重建。
    """
    try:
        cam.set_resolution(width=w, height=h)
    except Exception as exc:
        print("!! 切分辨率到 %dx%d 失败: %s" % (w, h, exc))
        return False
    try:
        cam.skip_frames(30)
    except Exception:
        pass
    return True


def run_digit_phase(cam, chassis_dev):
    """认数字。返回 1~4；认不出来返回 0。**跑完会把相机切回 CAM_W x CAM_H。**

    ⚠️ 标签陷阱：模型的 labels 是 ["3", "1", "2", "4"]，不是顺序的 1~4。
       所以必须用 detector.labels[class_id] 反查字符串再转 int，
       **绝不能写 class_id + 1** —— 那会把 3 当成 1、4 当成 2，停错点位。
    """
    print("=== 阶段 1: 识别数字 ===")

    if not _set_resolution(cam, DIGIT_CAM_W, DIGIT_CAM_H):
        return 0

    # 先试相对路径（模型跟应用打包在一起），不行再回退到系统目录
    model_path = DIGIT_MODEL_PATH_LOCAL
    if not os.path.exists(model_path):
        model_path = DIGIT_MODEL_PATH_SYS
    print("数字模型:", model_path, "存在" if os.path.exists(model_path) else "不存在!")

    try:
        detector = nn.YOLOv5(model=model_path)
    except Exception as exc:
        print("!! 数字模型加载失败，跳过识别:", exc)
        _set_resolution(cam, CAM_W, CAM_H)
        return 0

    print("模型标签:", detector.labels)

    last_digit = 0
    same_count = 0
    result = 0
    t_start = pytime.time()

    while not app.need_exit():
        if pytime.time() - t_start > DIGIT_MAX_SECONDS:
            print("!! 超时，没认出数字")
            break

        img = cam.read()
        try:
            objs = detector.detect(img, conf_th=DIGIT_CONF_TH, iou_th=DIGIT_IOU_TH)
        except Exception as exc:
            print("!! 检测失败:", exc)
            break

        digit = 0
        if len(objs) > 0:
            best = max(objs, key=lambda o: o.score)   # 取置信度最高的那个
            try:
                digit = int(detector.labels[best.class_id])
            except Exception:
                digit = 0

        if digit in (1, 2, 3, 4):
            same_count = same_count + 1 if digit == last_digit else 1
        else:
            same_count = 0
        last_digit = digit

        if same_count >= DIGIT_CONFIRM_N:
            result = digit
            print("确认数字 N = %d（连续 %d 帧）" % (digit, same_count))
            break

    _send_digit(chassis_dev, result)

    _set_resolution(cam, CAM_W, CAM_H)
    print("=== 阶段 2: 矩形识别瞄准 ===")
    return result


# =============================================================================
# §5 视觉流水线 —— 纯 cv2/numpy，不依赖 maix，可在 PC 上单测
# =============================================================================

# 各阶段耗时(ms)，由 detect() 填充
# find 那一栏太粗了（设备上占 11.8ms），拆成三块 —— 光靠猜已经错了三次，
# 先看清楚钱花在哪：找轮廓 / 拟合四边形 / 其余各道闸。
PROFILE_DATA = {
    "preprocess": 0.0, "find": 0.0, "homography": 0.0, "total": 0.0,
    "findcontours": 0.0, "approx": 0.0, "gates": 0.0,
    # child = largest_child_area（循环内，每个候选一次）
    # childq = _largest_child_quad（循环后，一次）—— 以前没被计时
    "child": 0.0, "childq": 0.0,
    # contours = 轮廓总数；cand = 过了面积/周长门槛、真正开始走 8 道闸的候选数。
    # 帧率随场景波动很大（同一份参数能差好几倍），所以需要一个【跟场景无关】的
    # 指标来判断 MIN_AREA 到底挡掉了多少 —— 这两个数就是干这个的。
    "contours": 0, "cand": 0,
    # 漏斗：过面积/过周长/过贴合/过几何/过长宽比/过填充/过暗区 各剩几个
    "funnel": (0, 0, 0, 0, 0, 0, 0),
}


# 形态学核缓存。原来是【每帧】调一次 getStructuringElement 造一个新核 ——
# 核的内容只跟尺寸有关，尺寸不变就该一直用同一个。纯浪费，且完全不改结果。
_KERNELS = {}


def _get_kernel(size):
    """取一个 size x size 的矩形核，按尺寸缓存。"""
    k = _KERNELS.get(size)
    if k is None:
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))
        _KERNELS[size] = k
    return k


def preprocess(frame_rgb):
    """灰度 -> 高斯模糊 -> 反二值化 -> 闭运算。返回 combined 二值图。

    输入是 RGB（image.image2cv 默认返回 RGB，不是 BGR）。
    THRESH_BINARY_INV 把黑色靶框翻成白色前景，供 findContours 使用。
    具体用哪种二值化由 THRESH_MODE 决定，见配置区。
    """
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (GAUSSIAN_BLUR_SIZE, GAUSSIAN_BLUR_SIZE), 0)
    #过滤掉高频噪点，保留低频大轮廓

    # --- 通道 A：区域（哪些地方够黑）---
    if THRESH_MODE == "fixed":
        # 只问"这个像素绝对够黑吗"。中灰背景进不来，不会误识别背景矩形。
        # 代价是保守 —— 反光处会被漏掉，靠下面的 Canny 和闭运算补。
        _, region = cv2.threshold(
            blur, FIXED_THRESHOLD, 255, cv2.THRESH_BINARY_INV
        )
    elif THRESH_MODE == "otsu":
        # 每帧自己算一个全局阈值（按灰度直方图找最佳分界），能跟着场地灯光走
        _, region = cv2.threshold(
            blur, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
        )
    else:
        # 自适应：只问"比周围暗吗"。光照免疫，但背景纹理会被大量收进来
        block = ADAPTIVE_BLOCK_SIZE if ADAPTIVE_BLOCK_SIZE % 2 == 1 else ADAPTIVE_BLOCK_SIZE + 1
        # block 必须为奇数，opencv 会自动修正为奇数，但这里显式保证。
        region = cv2.adaptiveThreshold(
            blur,                        # ① 输入：上一步模糊后的灰度图
            255,                         # ② 判定为"黑"的像素，在输出图里填几
            cv2.ADAPTIVE_THRESH_MEAN_C,  # ③ 用什么方法算阈值
            cv2.THRESH_BINARY_INV,       # ④ 正向还是反向
            block,                       # ⑤ 邻域大小
            ADAPTIVE_C,                  # ⑥ 偏移量
        )

    # --- 闭运算：先补区域通道的断口 ---
    # ⚠️ 顺序很关键：**先闭运算，后 OR 合并**（江南也是这个顺序）。
    # 如果反过来先 OR 再闭运算，大核会把 Canny 的 1 像素细边也加粗成十几像素的
    # 带子 —— 背景物体的一条边于是变成"有内孔的框"，形状和靶框一模一样，
    # 实测四边形面积会撑到真值的近两倍（113303 vs 约 66000）。
    # 先闭后并的话，大核只作用在阈值区域上，Canny 的细边保持细，
    # 细线围出的轮廓面积接近 0，会被 MIN_AREA 直接筛掉。
    if CLOSE_KERNEL_SIZE and CLOSE_KERNEL_SIZE >= 3:
        region = cv2.morphologyEx(region, cv2.MORPH_CLOSE, _get_kernel(CLOSE_KERNEL_SIZE))

    # --- 通道 B：从【闭合后的二值图】取轮廓 ---
    # ⚠️ Canny 的输入是 region（二值图），不是 blur（灰度图）。
    # 在灰度图上取边会把背景的每一条纹理都描出来 —— 那些线围出一堆碎轮廓，
    # 每个都要走一遍后面 8 道闸，实测杂乱场景能到 170 个。二值图上没有纹理，
    # 只有"够黑的块"，取出来的边自然干净。
    #
    # 补断口是【闭运算】的活，不是 Canny 的 —— Canny 不改变白色的形状，
    # 它只是把块描成一圈 1 像素的线。
    if USE_CANNY:
        combined = cv2.Canny(region, CANNY_LOW, CANNY_HIGH)
    else:
        combined = region

    # 返回两路：combined 用来找轮廓；region 留着给 find_target_rect 做
    # "这个轮廓是不是压在暗区上"的检查
    return region, combined


def _side_lengths(corners):
    """四条边长，按 corners 的给定顺序首尾相连。"""
    return [
        float(np.linalg.norm(corners[(i + 1) % 4] - corners[i])) for i in range(4)
    ]


def check_rectangle_geometry(corners):
    """几何校验：凸性 + 四角接近 90 度 + 对边等长 + 尺度合理。

    角度判据说明：拐角处两条边向量的夹角对矩形是 90 度(cos=0)，容差 30 度
    对应 |cos| <= sin(30) = 0.5，即菱形（角度偏离）与梯形（对边不等）都会被挡。
    """
    corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)

    sides = _side_lengths(corners)
    if min(sides) < MIN_SIDE_PX:
        return False

    # 凸性：相邻边叉积同号，挡掉自交的"蝴蝶结"四边形
    signs = []
    for i in range(4):
        e1 = corners[(i + 1) % 4] - corners[i]
        e2 = corners[(i + 2) % 4] - corners[(i + 1) % 4]
        signs.append(e1[0] * e2[1] - e1[1] * e2[0])
    if not (all(s > 0 for s in signs) or all(s < 0 for s in signs)):
        return False

    cos_limit = float(np.sin(np.radians(ANGLE_TOLERANCE_DEG)))
    for i in range(4):
        v1 = corners[(i - 1) % 4] - corners[i]
        v2 = corners[(i + 1) % 4] - corners[i]
        n1 = float(np.linalg.norm(v1))
        n2 = float(np.linalg.norm(v2))
        if n1 < 1e-6 or n2 < 1e-6:
            return False
        if abs(float(np.dot(v1, v2)) / (n1 * n2)) > cos_limit:
            return False

    for a, b in ((0, 2), (1, 3)):
        hi = max(sides[a], sides[b])
        if hi <= 0 or (hi - min(sides[a], sides[b])) / hi > SIDE_RATIO_TOLERANCE:
            return False

    # 最长边/最短边 <= 10（江南有这一条，补上）。
    # 它主要挡细长条；大部分情况已被上面的长宽比检查覆盖，留着是白送的。
    if max(sides) / min(sides) > MAX_SIDE_ASPECT:
        return False

    return True


def quad_fit_ratio(contour, area):
    """轮廓有多"贴合"一个矩形：面积 / 最小外接矩形面积。

    完整矩形 = 1.00 ；圆 ≈ 0.785 ；歪扭的碎块更低。

    ⚠️ 用 minAreaRect（带旋转）而不是轴对齐外接矩形 —— 后者对旋转极其敏感，
    矩形转 45° 时会掉到 0.5，等于把转着角度的靶纸自己杀掉。
    实测 minAreaRect 版本在 0~60° 下都是 0.987~1.000。

    这是"便宜的形状预筛"，插在 approximate_quad 之前用，见 MIN_QUAD_FIT。

    area 由调用方传进来 —— 它刚算过，别在这里再算一遍（cv2.contourArea 是
    遍历轮廓所有点的，白算一次就是白遍历一次）。
    """
    (_, (w, h), _) = cv2.minAreaRect(contour)
    box = float(w) * float(h)
    if box <= 0:
        return 0.0
    return area / box


def approximate_quad(contour, perimeter, original_area):
    """多 epsilon 试探，取"近似后面积最接近原轮廓"的 4 边形。失败返回 None。

    先取凸包再去掉凸性噪声点，避免斜视时轮廓上的毛刺把四边形带偏。

    original_area 由调用方传进来 —— 它刚算过，别在这里再算一遍
    （cv2.contourArea 要遍历轮廓所有点）。
    """
    if original_area <= 0 or perimeter <= 0:
        return None

    hull = cv2.convexHull(contour)
    best = None
    best_score = None

    for ratio in APPROX_EPSILON_CANDIDATES:
        approx = cv2.approxPolyDP(hull, ratio * perimeter, True)
        if len(approx) != 4:
            continue
        area_ratio = cv2.contourArea(approx) / original_area
        if area_ratio < APPROX_AREA_RATIO_LO or area_ratio > APPROX_AREA_RATIO_HI:
            continue
        score = abs(1.0 - area_ratio)
        if best_score is None or score < best_score:
            best, best_score = approx, score

    return best


def sort_corners(corners):
    """极角排序，统一为 左上 -> 右上 -> 右下 -> 左下。

    取"最接近 225 度"的点作起点（图像坐标系 y 向下，225 度即左上方向），
    再从该点沿角度递增方向取满四个点。比 sum/diff 启发式稳健：
    后者在靶纸旋转超过约 45 度时会排错角点，导致逆透视出 90 度翻转的图。
    """
    pts = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    centroid = pts.mean(axis=0)

    angles = np.arctan2(pts[:, 1] - centroid[1], pts[:, 0] - centroid[0])
    angles = (angles + 2.0 * np.pi) % (2.0 * np.pi)
    order = np.argsort(angles)

    target = 5.0 * np.pi / 4.0
    best_i, best_d = 0, float("inf")
    for i, idx in enumerate(order):
        d = abs(angles[idx] - target)
        d = min(d, 2.0 * np.pi - d)
        if d < best_d:
            best_d, best_i = d, i

    picked = [order[(best_i + k) % 4] for k in range(4)]
    return pts[picked]


# find_target_rect / frame_fill_ratio / ring_overlap_ratio 都要一张和 combined 同尺寸的
# 临时画布。原来每帧 np.zeros 一张（416x260 = 108KB，还要清零），这里缓存下来反复用。
# 这几个函数都是在同一帧内顺序调用、用完即弃，所以共用一张不会互相干扰。
_SCRATCH = None


def _get_scratch(shape):
    """取一张 shape 大小的临时画布；尺寸对不上才重新分配。"""
    global _SCRATCH
    if _SCRATCH is None or _SCRATCH.shape != tuple(shape):
        _SCRATCH = np.zeros(shape, dtype=np.uint8)
    return _SCRATCH


def find_target_rect(combined, region=None):
    """从二值图里找出靶纸外框。

    返回 (outer_quad, inner_quad_or_None, outer_area, label)，四角均已排序。
    inner_quad 仅供绘制，不参与误差计算（靶心由外框的单应矩阵算出）。
    label 固定为 "frame"；若返回 None 表示没找到。

    流程：初筛面积/周长 -> 四边形近似 -> 几何校验 -> 长宽比 -> 填充占比
          -> 取面积最大的框。
    填充占比（frame_fill_ratio）是抗干扰核心，负责把实心暗块挡在外面；
    取最大是因为靶纸是本场景视野里的主导物体。
    """
    t_fc = pytime.perf_counter()
    contours, hierarchy = cv2.findContours(
        combined, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
    )
    t_fc_end = pytime.perf_counter()
    PROFILE_DATA["findcontours"] = (t_fc_end - t_fc) * 1000.0
    PROFILE_DATA["contours"] = len(contours)
    if hierarchy is None or len(contours) == 0:
        PROFILE_DATA["approx"] = 0.0
        PROFILE_DATA["gates"] = 0.0
        PROFILE_DATA["cand"] = 0
        return None
    hierarchy = hierarchy[0]  # 每行 [next, prev, child, parent]

    scratch = _get_scratch(combined.shape)
    grown_region = None     # 真要判的时候才膨胀一次，见 ring_overlap_ratio
    frames = []
    approx_sec = 0.0        # 只累计 approximate_quad 自己的时间
    child_sec = 0.0         # 只累计 largest_child_area 自己的时间
    n_cand = 0              # 过了面积+周长门槛、开始走后面 8 道闸的候选个数
    # 各道闸过掉多少个（用来定位耗时：3 个贵操作各被调了几次）
    na = np0 = nf = ng = nasp = nfill = nring = 0
    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
        na += 1                                     # arcLength 被调用的次数
        perimeter = cv2.arcLength(contour, True)
        if perimeter < MIN_PERIMETER:
            continue
        np0 += 1                                    # minAreaRect 被调用的次数

        # 便宜的形状预筛 —— 插在 approximate_quad 之前。
        # 碎块在这里就被扔掉：后面的凸包、7 次 approxPolyDP、以及那 8 道闸
        # （设备实测每个候选约 100us）一次都不跑。见 MIN_QUAD_FIT。
        if quad_fit_ratio(contour, area) < MIN_QUAD_FIT:
            continue

        n_cand += 1
        nf += 1                                     # approximate_quad 被调用的次数

        ta = pytime.perf_counter()
        approx = approximate_quad(contour, perimeter, area)
        approx_sec += pytime.perf_counter() - ta
        if approx is None:
            continue

        quad = approx.reshape(4, 2).astype(np.float32)
        if not check_rectangle_geometry(quad):
            continue
        ng += 1

        _, _, w, h = cv2.boundingRect(approx)
        if h == 0:
            continue
        aspect = w / float(h)
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue
        nasp += 1

        # "是框不是块"的判据，见 frame_fill_ratio() 的说明。
        # ⚠️ 必须拿 region（真正的暗区）去测，不能拿 combined ——
        # combined 是"Canny 描出来的一圈线"，那里实心块【中间是空的】，
        # 看上去和一个中空的框一模一样，这道闸就废了（实测实心块会被误检）。
        base = region if region is not None else combined
        if frame_fill_ratio(contour, base, scratch) > MAX_FRAME_FILL:
            continue
        nfill += 1

        # "这个框是压在暗区上的吗" —— 挡掉 Canny 描出来的背景矩形
        if region is not None and MIN_RING_OVERLAP > 0:
            if grown_region is None:        # 每帧只膨胀这一次（原来每个候选都做一遍）
                grown_region = cv2.dilate(region, _get_kernel(RING_GROW_SIZE))
            if ring_overlap_ratio(contour, grown_region, scratch) < MIN_RING_OVERLAP:
                continue
        nring += 1

        # 内外轮廓互相印证：过低 = 外框被背景污染撑大了（框粘连）；
        # 过高 = 框太细（显示器边框/画框类干扰）。
        # 只在"确实有内孔"时才能判；框断掉时外轮廓没有子轮廓，跳过。
        if (MIN_INNER_OUTER_RATIO > 0) or (MAX_INNER_OUTER_RATIO > 0):
            tc = pytime.perf_counter()
            child_area = largest_child_area(contours, hierarchy, idx)
            child_sec += pytime.perf_counter() - tc
            if child_area >= MIN_AREA:
                io_ratio = child_area / area
                if io_ratio < MIN_INNER_OUTER_RATIO:
                    continue
                if (MAX_INNER_OUTER_RATIO > 0) and (io_ratio > MAX_INNER_OUTER_RATIO):
                    continue

        frames.append({"idx": idx, "quad": quad, "area": area})

    # find 拆账：循环总时间 = 拟合四边形 + 找子轮廓 + 各道闸
    loop_ms = (pytime.perf_counter() - t_fc_end) * 1000.0
    PROFILE_DATA["approx"] = approx_sec * 1000.0
    PROFILE_DATA["child"] = child_sec * 1000.0
    PROFILE_DATA["gates"] = loop_ms - PROFILE_DATA["approx"] - PROFILE_DATA["child"]
    PROFILE_DATA["cand"] = n_cand
    # 逐步漏斗：过面积 -> 过周长 -> 过贴合 -> 过几何 -> 过长宽比 -> 过填充 -> 过暗区
    PROFILE_DATA["funnel"] = (na, np0, nf, ng, nasp, nfill, nring)

    if not frames:
        return None

    # 取面积最大的框。
    # 江南代码是按面积升序取第一对（等价于取最小），实测会把画面里更小的
    # 干扰框（细边框画框、显示器边框）误当成靶纸。本题场景中靶纸是视野里的
    # 主导物体，取最大更稳。
    outer = max(frames, key=lambda r: r["area"])
    # _largest_child_quad 在循环【之后】跑，以前完全没被计时 ——
    # find 的耗时减去那三块之后剩下的那几毫秒，就是它。
    tq = pytime.perf_counter()
    inner = _largest_child_quad(contours, hierarchy, outer["idx"])
    PROFILE_DATA["childq"] = (pytime.perf_counter() - tq) * 1000.0
    return (
        sort_corners(outer["quad"]),
        inner,
        outer["area"],
        "frame",
    )


def _local_tile(contour, shape, pad):
    """把轮廓裁到它自己的外接矩形（四周留 pad），并平移到局部坐标。

    为什么必须裁：这两个函数原来都在【整幅图】上做 drawContours + bitwise_and +
    countNonZero —— 每次调用要清零 108KB、再扫几遍整幅图。而设备实测这 6 次调用
    （2 个候选 × 3 个函数）就吃掉 9ms。裁到外接矩形后，小候选的运算量能少几十倍。
    结果完全一样，只是范围小了。
    """
    x, y, w, h = cv2.boundingRect(contour)
    H, W = shape[:2]
    x0 = max(0, x - pad); y0 = max(0, y - pad)
    x1 = min(W, x + w + pad); y1 = min(H, y + h + pad)
    return (x0, y0, x1, y1), np.ascontiguousarray(contour - (x0, y0), dtype=np.int32)


def frame_fill_ratio(contour, combined, scratch):
    """外框内部的前景占比。实心块接近 1，中空的框很小。

    这是"是框不是块"的判据 —— 填充外轮廓后统计内部前景占比，与内孔是否被切碎无关。

    只在轮廓自己的外接矩形里算，见 _local_tile()。
    """
    (x0, y0, x1, y1), local = _local_tile(contour, combined.shape, 2)
    tile = scratch[y0:y1, x0:x1]
    tile[:] = 0
    cv2.drawContours(tile, [local], -1, 255, thickness=cv2.FILLED)
    inside = cv2.countNonZero(tile)
    if inside <= 0:
        return 1.0
    crop = np.ascontiguousarray(combined[y0:y1, x0:x1])
    foreground = cv2.countNonZero(cv2.bitwise_and(crop, tile))
    return foreground / float(inside)


def ring_overlap_ratio(contour, grown_region, scratch):
    """轮廓自身有多少比例的像素，落在"阈值暗区"（或其紧邻）上。

    用来区分两类形状一模一样的东西：
      · 真靶框 —— 轮廓压在黑色的边框上，而框本身在阈值暗区里 -> 比例高
      · 背景矩形（窗户、门框、白板）—— 是 Canny 用梯度描出来的细线，
        那条线上并没有暗的东西 -> 比例低
    没有这道检查，Canny 会把整个房间的矩形都变成候选，而选框规则是"取最大"，
    于是一个窗户就能顶掉靶纸（表现为画面跳变）。

    ⚠️ grown_region 收的是**已经膨胀过**的 region，不是原始 region。
    膨胀由 find_target_rect 每帧只做一次；以前是在这个函数里做的，于是
    **每一个候选都要把整幅 416x260 重新膨胀一遍**（同一张图重复劳动）。
    结果完全一样，只是不再白做功。
    膨胀的理由：阈值通常只抓到黑框的一部分（反光处漏掉），膨胀一下才不会
    把框上没抓到的那些段误判成"不在暗区"。

    只在轮廓自己的外接矩形里算，见 _local_tile()。线宽是 3，笔迹会超出
    外接矩形 1 像素，所以 pad 取 3 兜住。
    """
    (x0, y0, x1, y1), local = _local_tile(contour, grown_region.shape, 3)
    tile = scratch[y0:y1, x0:x1]
    tile[:] = 0
    cv2.drawContours(tile, [local], -1, 255, thickness=3)
    ring = cv2.countNonZero(tile)
    if ring <= 0:
        return 0.0
    crop = np.ascontiguousarray(grown_region[y0:y1, x0:x1])
    hit = cv2.countNonZero(cv2.bitwise_and(crop, tile))
    return hit / float(ring)


def largest_child_area(contours, hierarchy, idx):
    """该轮廓最大子轮廓（内孔）的面积；没有"合格的"子轮廓则返回 0。

    关键：**子轮廓自己也必须是一个合格的四边形**才算数（江南也是这个要求 ——
    他们要求父子两个都出现在 valid_rects 里）。只查面积不查形状的话，一个
    形状完全不规则的大洞也能让比值通过。

    和 _largest_child_quad 一样用"扫描全部、看谁的 parent 是它"来找，
    不顺着 first_child 指针走 —— 那个指向兄弟链里哪一个并无保证。
    """
    best = 0.0
    for i in range(len(contours)):
        if int(hierarchy[i][3]) != idx:
            continue
        contour = contours[i]
        area = cv2.contourArea(contour)
        if area < MIN_AREA or area <= best:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        approx = approximate_quad(contour, perimeter, area)
        if approx is None:
            continue
        if not check_rectangle_geometry(approx.reshape(4, 2).astype(np.float32)):
            continue
        best = area
    return best


def _largest_child_quad(contours, hierarchy, idx):
    """取该轮廓最大的子轮廓（内孔），它比外沿干净，用作逆透视的基准。

    hierarchy 每行是 [next, prev, child, parent]，父索引在第 3 位。
    这里故意用"扫描全部轮廓、看谁的 parent 等于 idx"来找子节点，
    而不是顺着 next/prev 指针走 —— OpenCV 的 first_child 指向兄弟链里的哪一个
    并无保证，顺着指针走可能只碰到一个就停了（实测会漏掉内孔）。
    N 只有几十，性能无所谓，换来的是不会错。
    """
    best_quad = None
    best_area = 0.0
    for i in range(len(contours)):
        if int(hierarchy[i][3]) != idx:     # 父不是它
            continue
        contour = contours[i]
        area = cv2.contourArea(contour)
        if area < MIN_AREA or area <= best_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        approx = approximate_quad(contour, perimeter, area)
        if approx is None:
            continue
        quad = approx.reshape(4, 2).astype(np.float32)
        if not check_rectangle_geometry(quad):
            continue
        best_quad, best_area = quad, area
    return sort_corners(best_quad) if best_quad is not None else None


def build_homography(outer_quad):
    """双向透视矩阵。dst 按靶纸实际尺寸铺满，故校正空间里 1px 恒等于固定毫米数。"""
    src = np.asarray(outer_quad, dtype=np.float32).reshape(4, 2)
    dst = np.array(
        [
            [0, 0],
            [RECT_W - 1, 0],
            [RECT_W - 1, RECT_H - 1],
            [0, RECT_H - 1],
        ],
        dtype=np.float32,
    )
    return (
        cv2.getPerspectiveTransform(src, dst),
        cv2.getPerspectiveTransform(dst, src),
    )


def transform_point(point, matrix):
    """单点透视变换。"""
    src = np.array([[[float(point[0]), float(point[1])]]], dtype=np.float32)
    out = cv2.perspectiveTransform(src, matrix)[0][0]
    return float(out[0]), float(out[1])


def detect(frame_rgb):
    """完整流水线。返回结果字典；未找到靶纸时 found 为 False。"""
    t0 = pytime.perf_counter()
    region, combined = preprocess(frame_rgb)
    t1 = pytime.perf_counter()

    found = find_target_rect(combined, region)
    t2 = pytime.perf_counter()

    result = {
        "found": False,
        "combined": combined,
        "outer": None,
        "inner": None,
        "outer_area": 0.0,
        "center": None,
        "laser_corr": None,
        "err_x_px": 0.0,
        "err_x_mm": 0.0,
        "matrix": None,
        "strategy": "-",
    }
    if found is None:
        if PROFILE:
            PROFILE_DATA["preprocess"] = (t1 - t0) * 1000.0
            PROFILE_DATA["find"] = (t2 - t1) * 1000.0
            PROFILE_DATA["homography"] = 0.0
            PROFILE_DATA["total"] = (t2 - t0) * 1000.0
        return result

    outer, inner, outer_area, strategy = found

    # 优先拿"内孔"（白色面板本身）当基准，而不是黑框的外沿。
    # 原因：黑框外沿会挂东西 —— 三脚架、上方数字牌、胶带接头，它们和框在二值图里
    # 连成一个连通块，外轮廓就不再是干净矩形，approxPolyDP 拟合出的四边形会被
    # 那根杆子带歪（实拍表现为"矩形变形"）。
    # 内孔是面板的边界，外面挂什么都不影响它，永远是干净的矩形。
    # 面板中心和靶心重合（圆环就印在面板上），所以换基准不影响瞄准。
    ref_quad = inner if inner is not None else outer
    matrix, matrix_inv = build_homography(ref_quad)
    t3 = pytime.perf_counter()

    # 靶心：校正空间几何中心反变换回原图。
    # 这与四边形对角线交点是严格等价的（射影变换保直线），但只需一次矩阵乘法。
    center = transform_point((RECT_W / 2.0, RECT_H / 2.0), matrix_inv)

    # 激光点：物理标定常量，正变换到校正空间后与靶心比较
    laser_corr = transform_point((LASER_X_PX, LASER_Y_PX), matrix)

    err_x_px = (center[0] - LASER_X_PX) * ERR_X_SIGN#水平误差（像素）
    err_x_mm = (RECT_W / 2.0 - laser_corr[0]) * MM_PER_PX_RECT * ERR_X_SIGN

    result.update(
        {
            "found": True,
            "outer": outer,
            "inner": inner,
            "outer_area": outer_area,
            "center": center,
            "laser_corr": laser_corr,
            "err_x_px": err_x_px,
            "err_x_mm": err_x_mm,
            "matrix": matrix,
            "strategy": strategy,
        }
    )

    if PROFILE:
        PROFILE_DATA["preprocess"] = (t1 - t0) * 1000.0
        PROFILE_DATA["find"] = (t2 - t1) * 1000.0
        PROFILE_DATA["homography"] = (t3 - t2) * 1000.0
        PROFILE_DATA["total"] = (t3 - t0) * 1000.0

    return result


# =============================================================================
# §6/§7 绘制（MaixCAM2 屏幕）
# =============================================================================


def _draw_quad(img, quad, color):
    pts = np.asarray(quad, dtype=np.int32)
    for i in range(4):
        x1, y1 = int(pts[i][0]), int(pts[i][1])
        x2, y2 = int(pts[(i + 1) % 4][0]), int(pts[(i + 1) % 4][1])
        img.draw_line(x1, y1, x2, y2, color, thickness=2)


def _draw_cross(img, x, y, color, size=10, thickness=2):
    ix, iy = int(x), int(y)
    img.draw_line(ix - size, iy, ix + size, iy, color, thickness=thickness)
    img.draw_line(ix, iy - size, ix, iy + size, color, thickness=thickness)


def draw_result(img, result):
    """把检测结果叠到 maix 图像上。"""
    # 激光点始终画出来：它是物理标定的固定像素，是误差的基准。
    # 放在 if 之前 —— 没找到靶纸时也画，屏幕上至少能看到"激光指着哪"。
    _draw_cross(img, LASER_X_PX, LASER_Y_PX, image.COLOR_YELLOW, size=10)

    if not result["found"]:
        img.draw_string(8, 8, "NO TARGET", image.COLOR_RED, scale=1)
        return

    _draw_quad(img, result["outer"], image.COLOR_GREEN)
    if result["inner"] is not None:
        _draw_quad(img, result["inner"], image.COLOR_BLUE)

    cx, cy = result["center"]
    # 中央竖线：靶心处的短竖线，直观对应题目的判据
    img.draw_line(int(cx), int(cy) - 16, int(cx), int(cy) + 16, image.COLOR_RED, thickness=2)
    img.draw_circle(int(cx), int(cy), 3, image.COLOR_RED, thickness=-1)
    # 激光点到靶心的连线，长度即水平误差
    img.draw_line(
        int(cx), int(cy), int(LASER_X_PX), int(LASER_Y_PX), image.COLOR_YELLOW, thickness=1
    )

    err_mm = result["err_x_mm"]
    color = image.COLOR_GREEN if abs(err_mm) <= 30.0 else image.COLOR_RED
    img.draw_string(
        8,
        8,
        "err_x {:+.0f}px  {:+.1f}mm".format(result["err_x_px"], err_mm),
        color,
        scale=1,
    )
    # 外框像素面积：标定 MIN_AREA 时直接读这个数
    img.draw_string(
        8,
        28,
        "area {:.0f}  {}".format(result["outer_area"], result["strategy"]),
        image.COLOR_WHITE,
        scale=1,
    )


def bin_button_rect():
    """右上角"原图/二值图"按钮的位置 (x, y, w, h)。"""
    return (CAM_W - BIN_BTN_W - 4, 4, BIN_BTN_W, BIN_BTN_H)


def point_in_rect(point, rect):
    if point is None:
        return False
    x, y, w, h = rect
    return x <= point[0] <= x + w and y <= point[1] <= y + h


def draw_bin_button(img, show_binary):
    """按钮上的字表示"按下去会切到什么"。

    显示原图时写 BIN（点它切到二值图），显示二值图时写 CAM（点它切回原图）。
    """
    x, y, w, h = bin_button_rect()
    img.draw_rect(x, y, w, h, image.COLOR_WHITE, thickness=2)
    img.draw_string(
        x + 8, y + 7,
        "BIN" if not show_binary else "CAM",
        image.Color.from_rgb(255, 0, 255),
        scale=1,
    )


def draw_bin_overlay(img, frame_rgb, combined):
    """二值图模式下叠一行实时数字，方便一边瞄一边调阈值。

    关键是 "<T" 这个百分比：**低于阈值的像素占全画面的比例**。
    它就是"阈值抓到了多少东西"。
      · 接近 0%    → 阈值太低，什么都没抓到，画面全靠 Canny 撑着
      · 1% ~ 15%   → 正常（靶框本身在画面里也就占这几个百分点）
      · 几十个百分点 → 阈值太高，背景全进来了
    fg 是加上 Canny 之后最终的前景占比，通常比 <T 大一些。
    """
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    below = float(np.count_nonzero(gray < FIXED_THRESHOLD)) / gray.size * 100.0
    fg = float(np.count_nonzero(combined)) / combined.size * 100.0
    # 不做颜色判断 —— 靶纸在画面里占多大随距离变化，这个百分比没有绝对的好坏区间
    # （凑近拍时靶框本身就占 30%）。它的用处是：改 T 的时候盯着它看变化。
    img.draw_string(
        8, CAM_H - 20,
        "T={}  <T {:.1f}%  fg {:.1f}%".format(FIXED_THRESHOLD, below, fg),
        image.COLOR_WHITE, scale=1,
    )


def binary_to_maix_image(combined):
    """把 numpy 二值图转成能贴到屏幕上的 maix 图像。

    先转成三通道再转 —— 灰度图在 MaixPy 上贴图/画字容易出问题，
    这点开销只在调试模式才有，无所谓。
    """
    rgb = cv2.cvtColor(combined, cv2.COLOR_GRAY2RGB)
    return image.cv2image(rgb, bgr=False, copy=True)


def draw_thumbnail(img, frame_rgb, result):
    """把校正图缩略图贴到右下角。

    上板时**肉眼确认逆透视方向是否正确**的唯一手段：靶纸应被拉成正矩形，
    内部圆环应是正圆；若拉歪了或出现 90 度翻转，说明角点排序或标定出了问题。
    这一步要真的调 warpPerspective，所以只在 DEBUG_THUMBNAIL 打开时跑。
    """
    try:
        corrected = cv2.warpPerspective(frame_rgb, result["matrix"], (RECT_W, RECT_H))
    except cv2.error:
        return
    h, w = corrected.shape[:2]
    thumb_h = max(1, int(h * THUMBNAIL_W / float(w)))
    thumb = cv2.resize(corrected, (THUMBNAIL_W, thumb_h), interpolation=cv2.INTER_AREA)
    # 十字标在画布中心，即算出来的靶心位置
    cv2.drawMarker(thumb, (THUMBNAIL_W // 2, thumb_h // 2), (255, 0, 0),
                   cv2.MARKER_CROSS, 12, 1)
    try:
        img.draw_image(
            CAM_W - THUMBNAIL_W - 4,
            CAM_H - thumb_h - 4,
            image.cv2image(thumb, bgr=False, copy=True),
        )
    except Exception:
        pass


def format_profile_lines(loop_ms, fps, detailed):
    """拼屏幕上的那几行字。

    detailed=False → 只拼一行 "fps XX.X"（平时就这个，帧率要一直看着）
    detailed=True  → 拼两行，多带各阶段耗时（调试用，由 PROFILE 控制）

    loop_ms 是【真实帧周期】：从这一轮循环开头，到下一轮循环开头之间的一切
    （取流 + 检测 + 绘制 + disp.show + 串口 + 触摸）。fps 由它算出来。

    ⚠️ 显示的是【上一轮】测到的值 —— 这一轮的周期要等 disp.show() 之后才知道，
    而这张图在那之前就已经画出去了。差一帧，无所谓。

    ⚠️ 不要用 maix.time.fps()：它算的是"距上次调用它的间隔"，几十帧才调一次的话
    报出来会偏低十几倍（实测报 1.9~3.3，而真实是 40~75）。

    和 draw_profile 拆开是为了分开两件事：
      · 文字【多久重算一次】 —— 每 PROFILE_EVERY 帧，数字才不每帧乱跳
      · 文字【多久画一次】   —— 每帧。屏幕每帧都是新画面，不重画字就没了。
    中间那些帧直接复用上一次拼好的字符串。
    """
    if not detailed:
        # 平时就这一行 —— 帧率是操作时要一直看着的（转云台时想知道够不够快）
        return ["fps {:.1f}".format(fps)]
    return [
        "fps {:.1f}  loop {:.1f}ms".format(fps, loop_ms),
        "pre {:.1f}  find {:.1f}  det {:.1f}".format(
            PROFILE_DATA["preprocess"], PROFILE_DATA["find"], PROFILE_DATA["total"]
        ),
    ]


def draw_profile(img, lines):
    """把耗时表画到屏幕上。lines 是 format_profile_lines() 拼好的字符串。"""
    y = 48
    for text in lines:
        img.draw_string(8, y, text, image.COLOR_WHITE, scale=1)
        y += 18


def touch_to_image(tx, ty, disp_w, disp_h, img_w, img_h):
    """屏幕触摸坐标 -> 图像坐标。disp.show() 走 FIT_CONTAIN 等比缩放居中。"""
    scale = min(disp_w / float(img_w), disp_h / float(img_h))
    if scale <= 0:
        return None
    draw_w, draw_h = img_w * scale, img_h * scale
    off_x, off_y = (disp_w - draw_w) / 2.0, (disp_h - draw_h) / 2.0
    return (tx - off_x) / scale, (ty - off_y) / scale


# 触摸打点算出来的灰度值缓存。key 变了才重算（见 draw_touch_info）。
_TOUCH_SAMPLE = {"key": None, "value": 0, "mean": 0.0, "fg": False}


def draw_touch_info(img, frame_rgb, point):
    """触摸打点：显示该点灰度值、邻域均值与二值化判定，用于调阈值。

    ⚠️ 灰度只在**打点那一下**算一次，之后每帧只重画文字。
    原来每帧都做一次整幅 cvtColor —— 而 touch_point 一旦设上就不会清，
    意味着**点一下之后整场跑图都被这个调试显示拖着**。打点是标定用的临时
    动作，不该让它一直占用帧时间。
    """
    ix, iy = int(point[0]), int(point[1])
    h, w = frame_rgb.shape[:2]
    if ix < 0 or ix >= w or iy < 0 or iy >= h:
        return

    key = (ix, iy, FIXED_THRESHOLD, ADAPTIVE_C, THRESH_MODE)
    if key != _TOUCH_SAMPLE["key"]:
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        value = int(gray[iy, ix])
        half = max(1, ADAPTIVE_BLOCK_SIZE // 2)
        y0, y1 = max(0, iy - half), min(h, iy + half + 1)
        x0, x1 = max(0, ix - half), min(w, ix + half + 1)
        local_mean = float(gray[y0:y1, x0:x1].mean())

        # 判定必须跟当前用的二值化方式一致 —— 否则显示的是"自适应怎么说"，
        # 而实际跑的是"固定阈值怎么说"，两个答案不一样时会把人带偏。
        if THRESH_MODE == "fixed":
            is_foreground = value < FIXED_THRESHOLD
        else:
            is_foreground = value < local_mean - ADAPTIVE_C

        _TOUCH_SAMPLE.update(
            {"key": key, "value": value, "mean": local_mean, "fg": is_foreground}
        )

    # 用 from_rgb 而不是 COLOR_xxx 常量，避免依赖不确定存在的颜色名
    magenta = image.Color.from_rgb(255, 0, 255)
    img.draw_circle(ix, iy, 4, magenta, thickness=2)
    img.draw_string(
        8,
        CAM_H - 40,
        "({},{}) gray {} mean {:.0f} -> {}".format(
            ix, iy, _TOUCH_SAMPLE["value"], _TOUCH_SAMPLE["mean"],
            "FG" if _TOUCH_SAMPLE["fg"] else "BG"
        ),
        magenta,
        scale=1,
    )


# =============================================================================
# §8 主循环
# =============================================================================


def _print_frame_stats(cam):
    """读一帧，把亮度、灰度分布、当前二值化方式打出来。

    这几个数是标定 FIXED_THRESHOLD 的依据（黑框灰度应该落在最低那一档），
    所以**不管用自动曝光还是锁定曝光都得打** —— 尤其自动曝光下胶带灰度会动，
    更需要开机能看见这几个数。
    """
    try:
        frame = image.image2cv(cam.read(), False, True)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        mean = float(gray.mean())
        print("画面平均亮度: %.0f / 255" % mean)

        # 这里只**读**相机当前用的曝光值，不设它 —— 自动曝光模式下这个数会自己变，
        # 打出来是为了让你看得见它在干什么（是不是一直在调、大概在什么量级）。
        try:
            cur_us = cam.exposure(-1)
        except Exception:
            cur_us = None
        if cur_us:
            print("相机当前曝光: %s us（自动曝光自己选的）" % cur_us)

        if mean > 200:
            print("!! 画面过亮 —— 现场光太强，给靶纸挡挡光")
        elif mean < 40:
            print("!! 画面过暗 —— 给靶纸补光")
        else:
            print("亮度正常（100~200 之间比较理想）")

        # 灰度分布 —— 用来定 FIXED_THRESHOLD：黑框应该落在最低的那一档
        ps = (1, 5, 25, 50, 75, 95)
        vals = [float(np.percentile(gray, p)) for p in ps]
        print("画面灰度分布: " + "  ".join("%d%%=%.0f" % (p, v) for p, v in zip(ps, vals)))

        if THRESH_MODE == "fixed":
            print("二值化: 固定阈值 %d（比它暗的算黑框）" % FIXED_THRESHOLD)
        else:
            print("二值化: %s" % THRESH_MODE)
    except Exception as exc:
        print("亮度自检失败（不影响运行）:", exc)


def init_camera():
    """创建相机，并把亮度与灰度分布打出来。

    LOCK_EXPOSURE = False（当前设置）：**完全用相机自带的自动曝光**，
    本程序一行都不去设它。开机不用等收敛，也不依赖"上电那一下镜头对着哪儿"。

    LOCK_EXPOSURE = True：走下面的检测逻辑 —— 上电先让相机自己收敛，
    把它选出的值读出来钉住；EXPOSURE_US > 0 时则直接锁到那个数。
    """
    cam = camera.Camera(CAM_W, CAM_H)

    if not LOCK_EXPOSURE:
        print("曝光: 用相机自带的自动曝光（本程序不干预）")
        _print_frame_stats(cam)
        return cam

    try:
        # ① 先让自动曝光收敛，把【曝光】和【增益】两个值都读出来。
        #    两个都要读：切到手动模式之后增益是独立的一套，只锁曝光不锁增益，
        #    画面亮度会跳到另一个值上（之前 10000µs 直接过曝就是这个原因）。
        if EXPOSURE_US == 0:
            # 先跳过上电暂态再开始采样。
            # 原因：ISP 刚起来时读数还是它的默认值（实测读到过 32944µs，画面全白），
            # 而那个默认值可能恰好"稳定"超过 0.4 秒 —— 下面的收敛判据会把这种
            # 假稳定误认成收敛，把默认值锁死。等一秒让 ISP 真正开始工作。
            pytime.sleep(1.0)

            # 上电测光照：让自动曝光自己收敛，把它的值读出来
            print("上电检测光照（等自动曝光收敛，镜头请对准实际场景）…")
            last_e, last_g, stable = None, None, 0
            for _ in range(60):            # 最多等 6 秒
                pytime.sleep(0.1)
                cur_e = cam.exposure(-1)
                cur_g = cam.gain(-1)
                if cur_e is None:
                    continue
                same_e = (
                    last_e is not None
                    and abs(float(cur_e) - float(last_e)) <= max(1.0, float(last_e) * 0.02)
                )
                same_g = (
                    last_g is None or cur_g is None
                    or abs(float(cur_g) - float(last_g)) <= max(1.0, abs(float(last_g)) * 0.02)
                )
                if same_e and same_g:
                    stable += 1
                    if stable >= 4:        # 连续 4 次基本不变，算收敛
                        break
                else:
                    stable = 0
                last_e, last_g = cur_e, cur_g
            want = cam.exposure(-1)
            want_gain = cam.gain(-1)
        else:
            want = EXPOSURE_US
            want_gain = GAIN if GAIN >= 0 else cam.gain(-1)

        # ② 【关键】切到手动曝光模式。
        #    不切的话，ISP 的自动曝光会一直覆盖下面设的值 —— 实测"锁"了半天，
        #    其实跑的一直是自动曝光：设定 9994 却读回 9971；填 10 和填 10000
        #    画面平均亮度都是 255。不切手动，整个方案等于没做。
        try:
            cam.exp_mode(camera.AeMode.Manual)
            print("已切到手动曝光模式")
        except Exception as exc:
            print("!! 切手动曝光模式失败:", exc)
            print("   -> 下面设的值很可能不生效，会退化成【持续自动曝光】")
            print("   camera 里带 Ae/Mode 的属性:",
                  [a for a in dir(camera) if "Ae" in a or "Mode" in a])

        # ③ 曝光和增益**一起锁**。
        #    两个都要锁：切到手动模式之后增益是独立的一套，只锁曝光不锁增益，
        #    画面亮度会跳到另一个值上。
        #    GAIN > 0 时用配置里写死的值；否则用收敛时读到的那个增益。
        cam.exposure(want)
        if GAIN > 0:
            cam.gain(GAIN)
        elif want_gain:
            cam.gain(want_gain)

        # ④ 读回验证 —— 这一步才是"到底锁上没有"的判据。
        #    设定值和读回值差得远，就是没锁上，后面所有标定都白做。
        got_exp = cam.exposure(-1)
        got_gain = cam.gain(-1)
        print("曝光: 设定 %s us  ->  读回 %s us" % (want, got_exp))
        print("增益: 设定 %s  ->  现在读回 %s" % (GAIN if GAIN > 0 else want_gain, got_gain))
        try:
            if got_exp and float(want) > 0:
                if abs(float(got_exp) - float(want)) / float(want) > 0.05:
                    print("!! 曝光没锁住（差超过 5%）—— 十有八九是手动模式没切上")
                else:
                    print("曝光已锁住")
        except Exception:
            pass
    except Exception as exc:
        # 锁失败不退出 —— 降级成自动曝光继续跑，后面的亮度自检照样要打
        print("锁曝光失败（保持自动曝光）:", exc)

    _print_frame_stats(cam)

    return cam


def run_find_rects(cam=None):
    if cam is None:
        cam = init_camera()     # 单独跑本文件时自己建; 两阶段流程用外面那个
    disp = display.Display()
    serial_dev = init_uart()  #先定义摄像头，串口

    cv2.setUseOptimized(True)
    cv2.setNumThreads(2)  # AX630C 双核  做视觉算法加速

    ts = None
    if DEBUG_TOUCH:
        try:
            ts = touchscreen.TouchScreen()
        except Exception as exc:
            print("触摸屏不可用:", exc)

    touch_point = None
    frame_count = 0
    # 真实帧周期：从这一轮循环开头，到下一轮循环开头之间的一切，含 disp.show()。
    # 原来量的那个 frame_ms 只到"绘制完"为止、不含 disp.show()，拿它算 fps 会偏高，
    # 所以整个换掉了。
    loop_ms = 0.0
    fps_real = 0.0
    profile_text = []       # 屏幕耗时表的文字缓存，见下面主循环
    loop_hist = []          # 最近 FPS_AVG_N 轮的周期，用来把 fps 读数磨平
    t_prev = pytime.perf_counter()
    show_binary = False     # 右上角按钮切换：False=看原图，True=看二值图
    bin_btn_held = False        # 手指是否还按在某个按钮上（防止按住不放时反复触发）

    print("MaixCAM2 靶纸瞄准视觉启动")
    print("符号约定: err_x = 靶心 - 激光点; 正值 = 激光点在靶心左侧")
    print("串口:", "已连接" if serial_dev else "未连接(降级运行)")

    while not app.need_exit():#程序不被打断便循环运行
        # 计时用 t_prev / loop_ms（在循环末尾更新）—— 那才是含 disp.show 的完整周期
        img = cam.read()#读取一帧图像
        frame_rgb = image.image2cv(img, False, False)  # 返回 RGB，不是 BGR
        #cv库要求使用bgr，但是我们的maxipy使用的是rgb，所以这里不转换，直接使用rgb
        result = detect(frame_rgb) #检测图像，返回结果字典
        frame_count += 1   #帧数+1

        # --- 串口 ---
        if serial_dev is not None:#如果串口初始化成功
            try:
                if result["found"]:
                    serial_dev.write(
                        build_uart_frame(STATUS_TARGET_VALID, result["err_x_px"])
                    )#如果检测到靶纸，发送数据包，包含状态码和误差值
                else:
                    serial_dev.write(build_uart_frame(STATUS_NO_TARGET, 0))#如果没有检测到靶纸，发送状态码和误差值为0
            except Exception as exc:
                print("串口发送失败:", exc)
                serial_dev = None

        # --- 触摸：右上角按钮 / 打点取样 ---
        if ts is not None:
            try:
                tx, ty, pressed = ts.read()#尝试读取触摸屏坐标和按下状态
                if pressed:
                    pt = touch_to_image(
                        tx, ty, disp.width(), disp.height(), CAM_W, CAM_H
                    )#将触摸坐标转换为图像坐标
                    if DEBUG_BIN_BUTTON and point_in_rect(pt, bin_button_rect()):
                        if not bin_btn_held:     # 只在按下的那一瞬间切一次
                            show_binary = not show_binary
                            bin_btn_held = True
                            print("显示切换到:", "二值图" if show_binary else "原图")
                    else:
                        touch_point = pt
                else:
                    bin_btn_held = False
            except Exception:
                ts = None

        # --- 绘制 ---
        if show_binary and result["combined"] is not None:
            # 二值图模式：直接看算法眼里的世界 —— 黑框连不连续、断在哪，一目了然
            show_img = binary_to_maix_image(result["combined"])
            draw_bin_overlay(show_img, frame_rgb, result["combined"])
        else:
            # ⚠️ 这行不是绘制代码 —— 它是"原图模式下要显示的那张图"。
            # 少了它，下面 draw_bin_button / disp.show 用的 show_img 就没定义，
            # 直接 UnboundLocalError 崩掉。别注释它。
            show_img = img
            if DRAW_RESULT:
                draw_result(img, result)#绘制检测结果
            if touch_point is not None:
                draw_touch_info(img, frame_rgb, touch_point)#绘制触摸信息
            if DEBUG_THUMBNAIL and result["found"] and frame_count % THUMBNAIL_EVERY == 0:
                draw_thumbnail(img, frame_rgb, result)#缩略图信息包括校正后的靶纸（整张拍平后的图）

        # 屏幕上的字：文字每 PROFILE_EVERY 帧重算一次，但【每帧都要画】。
        #   · 每帧都画 —— 屏幕每帧都是新的相机画面，不重画就没了（会一闪一闪）
        #   · 每 15 帧才重算 —— 数字不每帧乱跳；中间那些帧直接用上次拼好的字符串
        # PROFILE = False 时只画 "fps XX.X" 这一行；True 时多两行各阶段耗时。
        if not show_binary:
            if frame_count % PROFILE_EVERY == 0 or not profile_text:
                profile_text = format_profile_lines(loop_ms, fps_real, PROFILE)
            draw_profile(img, profile_text)

        if DEBUG_BIN_BUTTON:
            draw_bin_button(show_img, show_binary)

        disp.show(show_img)

        # --- 真实帧周期：从上一轮末尾到这一轮末尾，也就是完整的一轮（含 disp.show）---
        now = pytime.perf_counter()
        inst_ms = (now - t_prev) * 1000.0
        t_prev = now
        # 单轮的周期会抖（取流、检测、送屏各自都在变），所以显示用最近 N 轮的平均。
        loop_hist.append(inst_ms)
        if len(loop_hist) > FPS_AVG_N:
            loop_hist.pop(0)
        loop_ms = sum(loop_hist) / len(loop_hist)
        fps_real = 1000.0 / loop_ms if loop_ms > 0 else 0.0

        # maix_time.fps() 必须【每帧】调一次才有意义 —— 它算的是"距上次调用它的
        # 间隔的倒数"。以前每 15 帧才调一次，所以它报的是真实帧率的 1/15
        # （实测报 1.9~3.3，而真实是 40~75）。这里每帧调，它才能当对照。
        fps_lib = maix_time.fps()

        # 控制台也打一份（每 PROFILE_EVERY*10 帧一次）。屏幕上的数字没法复制粘贴，
        # 而调帧率恰恰要看这几个数，所以必须有一份能贴出来的。
        # maix_time.fps() 一起打出来做对照 —— 两个数【应该基本相等】。
        # 要是差很多，说明对它调用频率的假设又错了。
        if PROFILE and frame_count % (PROFILE_EVERY * 10) == 0:
            print(
                "[PROFILE] fps %.1f (周期 %.1fms) | detect %.1fms"
                " = pre %.1f + find %.1f + h %.1f | 取流+绘制+show+串口 %.1fms"
                "   [maix_time.fps()=%.1f]"
                % (
                    fps_real, loop_ms,
                    PROFILE_DATA["total"],
                    PROFILE_DATA["preprocess"], PROFILE_DATA["find"],
                    PROFILE_DATA["homography"], loop_ms - PROFILE_DATA["total"],
                    fps_lib,
                )
            )
            print(
                "           find %.1fms = 找轮廓 %.1f + 拟合四边形 %.1f"
                " + 找子轮廓 %.1f + 各道闸 %.1f + 选完之后 %.1f | 轮廓 %d 个, 过门槛 %d 个"
                % (
                    PROFILE_DATA["find"], PROFILE_DATA["findcontours"],
                    PROFILE_DATA["approx"], PROFILE_DATA["child"],
                    PROFILE_DATA["gates"], PROFILE_DATA["childq"],
                    PROFILE_DATA["contours"], PROFILE_DATA["cand"],
                )
            )
            f = PROFILE_DATA["funnel"]
            print(
                "           漏斗: 面积%d -> 周长%d -> 贴合%d -> 几何%d"
                " -> 长宽比%d -> 填充%d -> 暗区%d"
                % f
            )

    print("退出")


# =============================================================================
# §9 离线自测 —— 在 PC 上跑：python no_canny.py <图片>
#     用于不连板子先验证逆透视方向与靶心定位是否正确
# =============================================================================


def _draw_result_cv2(frame_bgr, result):
    cv2.drawMarker(
        frame_bgr,
        (int(LASER_X_PX), int(LASER_Y_PX)),
        (0, 255, 255),
        cv2.MARKER_CROSS,
        20,
        2,
    )
    if not result["found"]:
        cv2.putText(frame_bgr, "NO TARGET", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)
        return
    cv2.polylines(frame_bgr, [result["outer"].astype(np.int32)], True, (0, 255, 0), 2)
    if result["inner"] is not None:
        cv2.polylines(frame_bgr, [result["inner"].astype(np.int32)], True, (255, 0, 0), 2)
    cx, cy = int(result["center"][0]), int(result["center"][1])
    cv2.line(frame_bgr, (cx, cy - 16), (cx, cy + 16), (0, 0, 255), 2)
    cv2.line(frame_bgr, (cx, cy), (int(LASER_X_PX), int(LASER_Y_PX)), (0, 255, 255), 1)
    cv2.putText(
        frame_bgr,
        "err_x {:+.0f}px {:+.1f}mm".format(result["err_x_px"], result["err_x_mm"]),
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )


def _imread_unicode(path):
    """cv2.imread 在 Windows 上读不了中文路径，走 imdecode 绕开。"""
    try:
        buf = np.fromfile(path, dtype=np.uint8)
    except OSError:
        return None
    if buf.size == 0:
        return None
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def _imwrite_unicode(path, img):
    """cv2.imwrite 同理，走 imencode + tofile。"""
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        return False
    buf.tofile(path)
    return True


def _offline_test(path):
    frame_bgr = _imread_unicode(path)
    if frame_bgr is None:
        print("读取失败:", path)
        return
    # 所有阈值都按 CAM_W x CAM_H 调，测试图先缩到这个尺度，
    # 否则 MIN_AREA 之类的判据在高分辨率截图上等于没生效
    if frame_bgr.shape[1] != CAM_W or frame_bgr.shape[0] != CAM_H:
        print("原图 %dx%d，缩放到 %dx%d 以匹配阈值"
              % (frame_bgr.shape[1], frame_bgr.shape[0], CAM_W, CAM_H))
        frame_bgr = cv2.resize(frame_bgr, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    result = detect(frame_rgb)
    if not result["found"]:
        print("未检测到靶纸")
    else:
        print("外框四角 (TL,TR,BR,BL):\n", np.round(result["outer"], 1))
        if result["inner"] is not None:
            print("内框四角:\n", np.round(result["inner"], 1))
        print("外框面积: %.0f px^2" % result["outer_area"])
        print("命中策略: %s" % result["strategy"])
        print("靶心(原图): (%.1f, %.1f)" % result["center"])
        print("激光点(校正空间): (%.1f, %.1f)" % result["laser_corr"])
        print("err_x = %+.1f px  = %+.2f mm" % (result["err_x_px"], result["err_x_mm"]))

    base = path.rsplit(".", 1)[0]
    _imwrite_unicode(base + "_binary.png", result["combined"])

    overlay = frame_bgr.copy()
    _draw_result_cv2(overlay, result)
    _imwrite_unicode(base + "_overlay.png", overlay)
    print("已保存:", base + "_binary.png", "和", base + "_overlay.png")

    if result["matrix"] is not None:
        warped = cv2.warpPerspective(
            frame_bgr, result["matrix"], (RECT_W, RECT_H)
        )
        cv2.drawMarker(warped, (RECT_W // 2, RECT_H // 2), (0, 0, 255),
                       cv2.MARKER_CROSS, 30, 2)
        _imwrite_unicode(base + "_corrected.png", warped)
        print("已保存:", base + "_corrected.png", "—— 确认靶纸被拉成正矩形")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        _offline_test(sys.argv[1])
    else:
        # 上电两阶段: 先认数字(内部切 416x416, 跑完切回), 再矩形识别瞄准
        _cam = init_camera()
        run_digit_phase(_cam, init_uart_chassis())
        run_find_rects(_cam)
