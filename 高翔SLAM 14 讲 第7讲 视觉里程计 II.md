# 高翔SLAM 14 讲 第7讲 视觉里程计 II

## 1.1 3D-2D: PnP(Perspective-n-Point)

Perspective-n-Point是求解3D点到2D点对运动的方法，它描述了当我们知道了n个3D空间点以及他们的投影位置时，如何估计相机所在的位姿。2D-2D的对极几何方法需要八个或八个以上的点对，且存在初始化，纯旋转和尺度的问题。 而如果两个图像中，其中一张特征点的3D位置已知，那么最少需要三个点对就可估计相机运动。

### 1.1.1 直接线性变换

考虑某个空间点 $P = [X, Y, Z, 1]$ 。 在图像 $I_1$ 中，投影到特征点 $x_1 = (u_1, v_1, 1)^T$ (以归一化平面齐次坐标， 既z轴坐标为1，去掉了内参矩阵 $K$ 的影响)  https://blog.csdn.net/weixin_38133509/article/details/85689838.

此时相机的位姿 $R, t$ 是未知的，我们定义一个增广矩阵 $[R|t]$ 为一个 3×4的矩阵，展开可得:
$$
s\left(
 \begin{matrix}
   u_1 \\ v_1 \\ 1\\
  \end{matrix} 
\right) =\left(
 \begin{matrix}
   t_1 & t_2 & t_3 &t_4\\
   t_5 & t_6 & e_7 &t_8\\
   e_9 & e_{10} & e_{11} &t_{12}
  \end{matrix} 
\right)\left(
 \begin{matrix}
   X \\ Y \\ Z\\1
  \end{matrix} 
\right) = 0
$$
用最后一行把 $s$ 消去，得到两个约束:
$$
u_1 = \frac{t_1X + t_2Y +t_3Z + t_4 }{t_9X + t_{10}Y +t_{11}Z + t_{12}} \quad \quad v_1 = \frac{t_5X + t_6Y +t_7Z + t_8 }{t_9X + t_{10}Y +t_{11}Z + t_{12}}
$$


定义 $T$ 的行向量:
$$
\vec t_1 = (t_1,t_2,t_3,t_4)^T,\vec t_2 = (t_5,t_6,t_7,t_8)^T, \vec t_3 = (t_9,t_{10},t_{11},t_{12})^T
$$
于是有:
$$
\vec t_1^TP - t_3^TPu_1=0 \\
\vec t_2^TP - t_3^TPv_1=0
$$
$\vec t$待求， 可以看到每个特征点提供两个关于 $\vec t$ 的线性约束,假设一共有$N$ 个特征点，可以列出线性方程组:

![image-20220511001452703](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220511001452703.png)

  由于 $\vec t$ 一共有12维，因此最少通过六对匹配点，即可实现矩阵 $T$ 的线性求解 --- 直接线性变换 (Direct Linear Transform, DLT)。 当匹配点大于6时可以用SVD等方法对超定方程求最小二乘解。



在 DLT求解过程中，其将 $\vec t$ 的元素视为12个没有联系的未知数，没有考虑旋转矩阵 $\in SO(3)$ 的限制，需要针对所求出来的 $T$  寻找一个近似最好的旋转矩阵

 



### 1.1.2 P3P

P3P仅使用三对匹配点的几何关系。它的输入数据为三对 $3D - 2D$ 匹配点，记 3D点为 $A, B, C$ , 已知它的世界坐标系下的坐标，不是相机坐标系下的坐标。2D点为 $a,b ,c$ , 其中小写字母代表的是大写字母在相机成像平面上的投影。此外P3P还需要一对验证点，已从可能的解中选出正确的一个, 即为 $D - d$. 相机光心为 $O$.

![image-20220511074106767](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220511074106767.png)



考虑 $Oab$ 与 $OAB$及另外两对之间的关系，利用余弦定理有:
$$
OA^2+ OB^2-2OA \cdot OB \cdot cos<a,b> = AB^2 \\
OB^2+ OC^2-2OB \cdot OC \cdot cos<b,c> = BC^2 \\
OA^2+ OC^2-2OA \cdot OC \cdot cos<a,c> = AC^2
$$
上式同除以 $OC^2$, 并且记 $x = OA/OC, y=OB/OC, v = AB^2/OC^2, uv = BC^2/OC^2, wv=AC^2/OC^2$, 得:
$$
x^2 + y^2 -2xycos<a,b> - v=0\\
1 + y^2 -2ycos<b,c> - uv=0\\
x^2 + 1 -2xcos<a,c> - wv=0\\
$$
 在1式中的v代入 2，3可得:
$$
(1-u)y^2-ux^2-cos<b,c>y+2uxycos<a,b>+1=0\\
(1-w)x^2-wy^2-cos<a,c>x+2wxycos<a,b>+1=0
$$
我们已知2D点的图像位置，三个余弦角是已知的，同时 $u=BC^2/AB^2, w=AC^2/AB^2$ 也可通过A,B,C在世界坐标系下的坐标算出，所以只有 $x,y$是未知的，二元二次方程，需要用到吴消元法，求得4个可能的解，并根据验证点得到正确的解，得到 $A,B,C$在相机坐标下的3D坐标，根据世界坐标到相机坐标的变换来计算相机的运动 $R, t$

P3P具体求解过程:https://www.cnblogs.com/mafuqiang/p/8302663.html

根据相机坐标与世界坐标求位姿: https://blog.csdn.net/u011426016/article/details/102969795





> P3P的问题:
>
> 1. P3P只利用三个点的信息，当给定的配对点多于3组时，难以利用更多的信息
> 2. 如果3D点或2D点受噪声影响，或者存在误匹配，则算法失效







### 1.1.3 Bundle Adjustment

在SLAM中我们通常先使用 P3P/EPnP的方法来估计相机位姿，然后构建最小二乘优化问题对估计值进行调整。

我们可以把PnP问题构建成一个定义于李代数上的非线性最小二乘问题。（**线性方法** 是指先求相机位姿，再求空间点位置， **非线性优化** 是将他们都看成优化变量，放在一起优化）



在PnP中，这个Bundle Adjustment问题是一个 **最小化重投影误差** 的问题。考虑 n个三维空间点$P$ 和他们的投影 $p$，设待求的相机位姿 $R,t$ 的李代数为 $\xi$。 像素位置与空间点位置关系如下:
$$
s_i\left[
 \begin{matrix}
   u_i \\ v_i \\ 1\\
  \end{matrix} 
\right] = K \exp(\xi ^{\land})\left[
 \begin{matrix}
   X_i \\ Y_i \\ Z_i \\ 1
  \end{matrix} 
\right]
$$
写成矩阵形式:
$$
s_i \vec u_i = K\exp (\xi^{\land})P_i
$$
注意这里 $K$ 是一个3×3的矩阵，因为该式中间隐藏着一个齐次坐标到非齐次坐标的变换。

现在由于 **观测点的噪声** 及 **相机位姿未知**， 该等式会存在一个误差，我们构建最小二乘问题以寻找最好的相机位姿，最小化误差,即像素坐标与3D点按照目前估计的位姿进行投影相比较所得到的误差，称之为 **重投影误差**:
$$
\xi^{*} = argmin_{\xi} \frac{1}{2}\sum_{i=1}^{n} ||u_i - \frac{1}{s_i}K\exp(\xi^{\land}P_i)||_2^2
$$

最小二乘优化可以通过Gauss-Netwon 与 Levenberg-Marquadt 求解，不过我们需要知道每个误差项关于优化变量的导数:
$$
e(x + \Delta x) \approx e(x) + J\Delta x
$$
推导 $J$ 的形式:

- 1. 首先记变换到相机坐标系下的空间点坐标为 $P^{'}$ , 取其前三维:
     $$
     P^{'} = (\exp(\xi^{\land})P)_{1:3} = [X^{'}, Y^{'}, Z^{'}]
     $$

- 2. 则相机投影模型为：
     $$
     su = KP^{'} \\
     \left[
      \begin{matrix}
       s u \\s v\\ s\\
       \end{matrix} 
     \right] = \left[
      \begin{matrix}
        f_x & 0 & c_x \\ 0 &  f_y & c_y\\ 0 & 0 & 1
       \end{matrix} 
     \right]\left[
      \begin{matrix}
        X' \\ Y' \\ Z'
       \end{matrix} 
     \right]
     $$
     解得s代入 1，2得：
     $$
     u = f_x\frac{X'}{Z'} + c_x, \quad v =  f_y\frac{Y'}{Z'} + c_y
     $$
     可以这里 得 u ,v与实际的测量值比较，求差。

  3.  我们对 $\xi^{\land}$ 左乘扰动量 $\delta \xi$ , 利用链式法则有:
     $$
     \frac{\partial e}{\partial \delta \xi} = \lim\limits_{\delta\xi\rightarrow\infty}\frac{e(\delta \xi \bigoplus\xi )}{x}	 = \frac{\partial e}{\partial P'}\frac{\partial P'}{\partial \delta \xi}
     $$

​		易得:
$$
\frac{\partial e}{\partial P'} = \left[
 \begin{matrix}
   \frac{\partial u}{\partial X'} & \frac{\partial u}{\partial Y'} & \frac{\partial u}{\partial Z'} \\ \frac{\partial v}{\partial X'} &  \frac{\partial v}{\partial Y'} & \frac{\partial v}{\partial Z'}
  \end{matrix} 
\right] = -  \left[
 \begin{matrix}
   \frac{f_x}{Z'} & 0 & -\frac{f_xX'}{Z'^2} \\ 0 &  \frac{f_y}{Z'} & -\frac{f_yY'}{Z'^2}
  \end{matrix} 
\right]
$$
第二项是李代数中的导数:
$$
\frac{TP}{\partial \delta \xi} = (TP)^{\odot} = \left[
 \begin{matrix}
  I & -P'^{\land} \\0^T & 0^T
  \end{matrix} 
\right]
$$
由于我们只取了P‘的前三维，所以得(3×6矩阵)：

> https://blog.csdn.net/qq_43256088/article/details/123550314

$$
\frac{\partial P'}{\partial \delta \xi}= [I,  -P'^{\land}]
$$
其中: 
$$
-P'^{\land} = \left[
 \begin{matrix}
   0 & -Z' & Y' \\ Z' &  0 & -X'\\ -Y' & X' & 0
  \end{matrix} 
\right], \quad I = \left[
 \begin{matrix}
   1 & 0 & 0 \\ 0 &  1 & 0\\ 0 & 0 & 1
  \end{matrix} 
\right]
$$


相乘得：
$$
\frac{\partial e}{\partial P'}\frac{\partial P'}{\partial \delta \xi} = -\left[
 \begin{matrix}
   \frac{f_x}{Z'} & 0 & -\frac{f_xX'}{Z'^2} & -\frac{f_xX'Y'}{Z'^2} & f_x + \frac{f_xX^2}{Z'^2}  & -\frac{f_xY'}{Z'}\\ 0 & \frac{f_y}{Z'}  & \frac{f_yY'}{Z'^2} & -f_y-\frac{f_y'Y'^2}{Z'^2} & \frac{f_yX'Y'}{Z'^2} & \frac{f_yX'}{Z'}
  \end{matrix} 
\right]
$$
这个 **2×6的雅可比矩阵** 描述了重投影误差关于相机位姿李代数的一阶变化关系，保留负号是因为误差是由观测值减去从姿态的估计值



另，我们想优化特征点的空间位置，因此有:
$$
\frac{\partial e}{\partial P} = \frac{\partial e}{\partial P'}\frac{\partial P'}{\partial P}
$$
又∵ $P' = \exp(\xi ^{\land})P = RP + t$ 

所以:
$$
\frac{\partial e}{\partial P} = \frac{\partial e}{\partial P'}\frac{\partial P'}{\partial P} = -  \left[
 \begin{matrix}
   \frac{f_x}{Z'} & 0 & -\frac{f_xX'}{Z'^2} \\ 0 &  \frac{f_y}{Z'} & -\frac{f_yY'}{Z'^2}
  \end{matrix} 
\right]R
$$




## 1.2 3D-3D: ICP方法

3D-3D的位姿估计问题，假设我们有一组配对好的3D点，（如匹配好的RGB-D图像）:
$$
P = \left\{p_1,...,p_n\right\}, \quad P' = \left\{p'_1,...,p'_n\right\}
$$
现在寻找一个欧式变换使得:
$$
\forall i, p_i = Rp_i'+t
$$
这个问题可以用迭代最近点 (iterative cloest point ICP)求解，且不用相机模型。在RGB-D相机中可以用这种方法估计相机位姿

### 1.2.1 SVD方法

即利用 **线性代数** 求解，我们定义第 $i$ 对点的误差项:
$$
e_i = p_i - (Rp'_i + t)
$$
构建最小二乘问题,求使误差平方差和最小的 R与t:
$$
min_{R,t} = \frac{1}{2}\sum_{i=1}^{n}||p_i - (Rp'_i+t)||^2_2
$$
先定义两组点的质心:
$$
p = \frac{1}{n}\sum_{i=1}^{n}(p_i), \quad p' = \frac{1}{n}\sum_{i=1}^{n}(p'_i)
$$


在误差函数中我们做如下处理:

![image-20220511185441458](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220511185441458.png)

观察左右两项，左边只与$R$有关，右边 $R,t$ 都有，但只与质心有关，因此我们求得 $R$ 并求得使右边等于0 的 $t$ 即可。

所以 ICP 的步骤可以写为:

1. 计算两组点的质心位置 $p,p'$，然后计算每个点的去质心坐标:
   $$
   q_i = p_i -p, \quad q'_i = p'_i -p'
   $$

2. 根据以下优化问题计算旋转矩阵:
   $$
   R^* =argmin_R\frac{1}{2}\sum_{i=1}^n||q_i - Rq'_i||^2
   $$
   展开可得:
   $$
   \frac{1}{2}\sum_{i=1}^n||q_i - Rq'_i||^2 = \frac{1}{2}\sum_{i=1}^n q_i^Tq_i + q'^T_iR^TRq'_i -2q'^T_iRq'_i
   $$
   第一项第二项与R没关系，所以目标函数变为:
   $$
   \sum_{i=1}^nq'^T_iRq'_i = \sum_{i=1}^n-tr(Rq'_iq^T_i)=-tr(R\sum_{i=1}^nq'_iq^T_i)
   $$
   

   ![image-20220511190538576](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220511190538576.png)







3. 根据求得的R计算t





### 1.2.2 非线性优化：

通过李代数扰动模型不断迭代
