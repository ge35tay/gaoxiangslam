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



在PnP中，这个Bundle Adjustment问题是一个 **最小化重投影误差** 的问题。考虑 n个三维空间点$P$ 和他们的投影 $p$，设待求的相机位姿 $R,t$ 的李代数为 $\xi$, 