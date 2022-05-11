# 高翔 SLAM 14讲 第8讲 视觉里程计 II

## 1.1 直接法

 特征法的几个缺点：

- 1. 关键点的提取与描述子的计算非常耗时。
  2. 使用特征点时，忽略了除特征点之外的所有信息，丢弃了大部分可能有用的图像信息
  3. 相机有时候会运动到 **特征缺失的地方**， 这些场景下特征点数量会明显减少，可能会找不到足够的匹配点来计算相机运动、



为了克服以上缺点解决方案有:

- 保留特征点，但只计算关键点，不计算描述子，同时用 **光流法** 来跟踪特征点的运动，这样可以回避计算和匹配描述子带来的时间，但光流本身的计算需要一点时间，即仍使用特征点，而估计相机运动时仍使用 对极几何，PnP或者ICP算法,用 **最小化重投影误差** 来优化相机运动, 需要精确的知道空间点在两个相机中投影后的像素位置，所以需要对特征点进行匹配和追踪
- 只计算关键点，不计算描述子，同时用 **直接法** 来计算特征点在下一时刻图像的位置，这同样可以跳过描述子的计算过程，且直接法计算更简单
- 既不计算关键点，也不计算描述子，而是根据像素灰度的差异，直接计算相机运动



后两种方法会根据图像的灰度信息来计算相机运动，称为 **直接法**，我们并不需要知道点与点的对应关系，而是通过 **最小化光度误差** 来求得他门，即只要场景存在明暗变化，直接法就能工作



## 1.2 光流

光流描述了像素在图像中的运动

![image-20220511214930229](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220511214930229.png)

计算部分像素运动的称为 **稀疏光流**， 而计算所有像素的称为 **稠密光流**



### 1.2.1 稀疏光流的代表 Lucas-Kanade光流 (LK光流)

在LK 光流中我们假设来自相机的图象是随时间变化的，图像看作是时间的函数 $I_t$ , 位于 $(x,y)$ 处的像素，其灰度可以写成: $I(x,y,t)$

我们做一个假设性很强的假设: **灰度不变假设**， 即:
$$
I(x+dx, y+dy,z+dz) =I(x,y,z)
$$
左边进行泰勒展开：
$$
I(x+dx, y+dy,z+dz) \approx I(x,y,t) + \frac{\partial I}{\partial x}dx + \frac{\partial I}{\partial y}dy + \frac{\partial I}{\partial t}dt
$$
∴：
$$
\frac{\partial I}{\partial x}dx + \frac{\partial I}{\partial y}dy + \frac{\partial I}{\partial t}dt = 0
$$
两边同时以 $dt$ 得:
$$
\frac{\partial I}{\partial x}\frac{dx}{dt} + \frac{\partial I}{\partial y}\frac{dy}{dt} = -\frac{\partial I}{\partial t}
$$
$dx/dt$为像素在 x轴上的运动速度， $dy/dt$ 为y轴速度，记为 $u,v$,  $\frac{\partial I}{\partial x}$ 为图像在该点 x方向的梯度， $\frac{\partial I}{\partial y}$  为图像在该点y方向的梯度，记为 $I_x, I_y$

写成矩阵形式有:
$$
\left[
 \begin{matrix}
   I_x & I_y
  \end{matrix} 
\right]\left[
 \begin{matrix}
   u\\ v
  \end{matrix} 
\right] = -I_t
$$
考虑一个大小为 $w×w$ 大小的窗口，它含有$w^2$ 个像素，所以我们共有 $w^2$ 个方程:
$$
\left[
 \begin{matrix}
   I_x & I_y
  \end{matrix} 
\right]_k\left[
 \begin{matrix}
   u\\ v
  \end{matrix} 
\right] = -I_{tk}, \quad k =1,..., w^2
$$
记:
$$
A = \left[
 \begin{matrix}
   \left[
 \begin{matrix}
   I_x & I_y
  \end{matrix} 
\right]_1\\ . \\ . \left[
 \begin{matrix}
   I_x & I_y
  \end{matrix} 
\right]_k
  \end{matrix} 
\right] \quad b=\left[
 \begin{matrix}
   I_{t1}\\ . \\ . \\ I_{tk}
  \end{matrix} 
\right]
$$
可得:
$$
A \left[
 \begin{matrix}
   u\\ v
  \end{matrix} 
\right] = -b
$$
对于此超定方程，我们常用最小二乘法求解:
$$
\left[
 \begin{matrix}
   u\\ v
  \end{matrix} 
\right]^{*} = -(A^TA)^{-1}A^Tb
$$
这样就可以得到像素在图像中的运动速度 $u, v$ , 可以根据此估计某块像素在若干个图像图像出现的位置





## 1.3 直接法 (Direct Methods)

 ![image-20220511222421219](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220511222421219.png)

### 1.3.1 直接法的推导

考虑某个空间点 P和两个时刻的相机， $P$ 的世界坐标为 $[X,Y,Z], 其在非齐次像素坐标为:  

$p_1, p_2$

列出完整的投影方程:
$$
p_1 = \left[
 \begin{matrix}
   u\\ v \\1
  \end{matrix} 
\right]_1 = \frac{1}{Z_1} KP, \\
p_2 = \left[
 \begin{matrix}
   u\\ v \\1
  \end{matrix} 
\right]_2 = \frac{1}{Z_2} K(RP + t) = \frac{1}{Z_2} K(\exp(\xi^{\land})P))_{1:3}
$$


$Z_1$ 是 $P$ 的深度， $Z_2$  是 $P$ 在第二个相机坐标下的深度。在特征点法中，我们通过匹配描述子知道了 $p_1, p_2$ 的像素位置，可以计算重投影的位置。但是在直接法中我们不知道哪一个 $p_2$ 与 $p_1$ 对应。

直接法的思路是根据当前相机的位姿估计值，来寻找 $p_2$ 的位置，如果位姿不够好， $p_2$ 外观会和 $p_1$ 有明显差别，为了减小这个差别，需要优化位姿，此时最小化的不是重投影误差而是 **光度误差** ,也就是P两个像的亮度误差:
$$
e = I_1(p_1) - I_2(p_2)
$$
优化目标:
$$
min_{\xi}J(\xi) = ||e||^2 = \sum_{i=1}^N e_i^Te_i, \quad e_i = I_1(p_{1,i}) - I_2(p_{2,i})
$$
为了求解这个问题，我们来看误差是如何随着相机位姿 $\xi$ 变化的，使用李代数上的左扰动模型:
$$
e(\xi \oplus \delta \xi) = I_1(\frac{1}{Z_1}KP) - I_2(\frac{1}{Z_2}K \exp(\delta \xi ^{\land})\exp( \xi ^{\land}) P)\\
\approx I_1(\frac{1}{Z_1}KP) - I_2(\frac{1}{Z_2}K (1 + \delta \xi^{\land})\exp( \xi ^{\land}) P) \\
= I_1(\frac{1}{Z_1}KP) - I_2(\frac{1}{Z_2}K\exp( \xi ^{\land}) P + \frac{1}{Z_2}K\delta \xi^{\land}\exp( \xi ^{\land}) P)
$$
记:
$$
q = \delta \xi^{\land}\exp( \xi ^{\land}) P\\
u = \frac{1}{Z_2}Kq
$$
$q$ 为 P在扰动后，位于第二个相机坐标系下的坐标，而 $u$ 为它的像素坐标

利用一阶泰勒展开有:
$$
e(\xi \oplus \delta \xi) =  I_1(\frac{1}{Z_1}KP) - I_2(\frac{1}{Z_2}K\exp( \xi ^{\land}) P + u) \\
\approx  I_1(\frac{1}{Z_1}KP) - I_2(\frac{1}{Z_2}K\exp( \xi ^{\land}) P) - \frac{\partial I_2}{\partial u}  \frac{\partial u}{\partial q} \frac{\partial q}{\partial \delta \xi}\delta \xi \\
= e(\xi) - \frac{\partial I_2}{\partial u}  \frac{\partial u}{\partial q} \frac{\partial q}{\partial \delta \xi}\delta \xi
$$
可以看到：

![image-20220511234507503](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220511234507503.png)







直接法的优缺点:

- 可以省去计算特征点，描述子的时间
- 只要求有像素梯度即可
- 可以构建半稠密乃至稠密地图

缺点：

- 非凸性，直接法完全依靠梯度搜索，降低目标函数来计算位姿
- 单个像素没有区分度，只能少数服从多数
- 灰度不变是很强的假设
