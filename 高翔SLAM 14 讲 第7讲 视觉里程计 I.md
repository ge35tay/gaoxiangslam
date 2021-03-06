# 高翔SLAM 14 讲 第7讲 视觉里程计 I

SLAM 前端也被称为视觉里程计 (VO) ,  它根据相邻图像的信息来粗略估计相机的运动，给后端优化提供较好的初始值， VO的算法按照需不需要提取特征分为特征点法的前端与不提特征的直接法前端

## 1.1 特征点法

基于特征点法的前端运行稳定，对光照动态物体不敏感，是目前比较成熟的方案

### 1.1.1 特征点

 根据图像估计相机运动： 首先在图像中提取比较有代表性的点，这些点在相机视角发生少量变化后会保持不变，所以我们可以从不同的图像中找到相同的点，然后在这些点的基础上讨论相机位姿估计问题，以及这些点的定位问题。在经典SLAM模型中这些点称为 **路标**，在视觉SLAM模型中称为 **特征点**



特征是图像信息的另一种数字表达形式，我们希望特征在相机运动后保持稳定。 

- 最简单的特征是单个 ==图像像素的灰度值==， 但是单个图像像素受光照、形变、物体材质的影响变化非常大，非常不稳定。



- 常用的特征点为 **角点，边缘，区块 **。
  - 特征点需要的性质
    - 1. **可重复性**: 相同的区域可以在不同的图像中被找到
      2. **可区别性**： 不同的区域有不同的表达
      3. **高效率**:  同一个图像中，特征点的数量应该远小于像素的数量
      4. **本地性**： 特征仅与一小片图像区域相关
    - 特征点由 **关键点** 与 **描述子** 两部分组成，关键点是指该特征在图像里的位置，有些特征点还具有方向、大小等信息，描述子是一个向量，按找某种人为设计的方式(按照 "外观相似的特征应该有相似的的描述子"设计)，描述了该关键点周围像素的信息。提取关键点并计算描述子。
    - 
1. FAST关键点
   
    FAST检验的关键点是角点，主要是检测局部像素灰度变化明显的地方，以速度快著称。
    
    idea: 如果一个像素与它邻域的像素差别较大(过亮或过暗)，那他更可能是角点
    
    > 检测过程
    >
    > 1. 在图像中选取像素 $p$，假设它的亮度为$I_p$
    >
    > 2. 设置一个阈值 $T$ (比如 $I_p$ 的 20%)
    >
    > 3. 以像素 $p$ 为中心，选取半径为3的圆上的16个像素点
    >
    > 4. 假如选取的圆上，有连续的 $N$ 个点亮度大于 $I_p + T$ 或小于 $I_p - T$ , 那么像素 $p$ 可以被认为是特征点， (N常取9, 11, 12 分别对应 FAST-9, FAST-11, FAST-12)
    >
    > 5. 对图中每一个像素执行相同的操作
    >
    >    
    >
    > 为了更高效的运行FAST可以添加一个预测试操作，以快速排除绝大多数不是角点的像素: 
    >
    > 对每个像素，直接检测邻域圆上的第1, 5, 9, 13个像素的亮度，只有当这四个像素中有三个同时大于 $I_p + T$ 或小于 $I_p - T$ 时，此时像素才有可能是一个角点，否则直接排除；
    >
    > 
    >
    > 此外为了常在FAST第一遍检测后添加非极大值抑制防止角点扎堆
    
    
    
2. 基于 FAST的ORB算法
   

ORB的关键点时oriented FAST, ORB计算了特征点的主方向，为后续的BRIEF描述子增加了旋转不变特性。
        
- FAST特征点数量很大且不确定，而且往往希望对图像提取固定数量的特征点，因此，在ORB中会计算原始FAST角点的Harris响应值，并选取前 $N$ 个具有最大响应值的角点，作为最终的角点集合。
  
 - FAST角点不具有方向信息，而且由于其固定选择半径为3的圆，存在尺度问题，远看像角点的地方可能靠近后就不是角点了。针对这两点ORB添加了尺度和旋转的描述。
        
	尺度不变性由构建图像金字塔，即最底层图象是图像最原始分辨率，上面几层是把图像缩小一倍，两倍等，在金字塔不同层分别检测提取FAST角点，获取各个尺度下的FAST特征
        
	旋转是用 **灰度质心法** 实现的
   
    
   
    

	灰度质心：
   
    - 质心是以图像块灰度值作为权重的中心，在一个图像块 $B$ 中定义图像块的矩为：
      $$
      m_{pq} = \sum_{x,y \in B} x^py^qI(x,y), \quad \quad p,q = \{0, 1\}
      $$
      
    - 通过矩可以找到图像块的质心:
      $$
      C = (\frac{m_{10}}{m_{00}},\frac{m_{01}}{m_{00}} )
      $$
      
    - 连接图像块的几何中心 $O$ 与质心 $C$ ,得到一个方向向量 $\vec{OC}$ , 于是特征点的方向可以定义为:
      $$
      \theta = \arctan(m_{01}/m_{10})
      $$
      通过以上方法FAST角点就有了尺度与旋转的描述


​    
​    
3. BRIEF描述子
   

https://senitco.github.io/2017/07/05/image-feature-brief/
    
在提取 Oriented FAST关键点后，对每个点计算其描述子。BRIEF是用二进制计算描述子，他的描述向量由许多个0或1组成。对于特征点附近的两个点 $p$ 和 $q$ ， 比较它们的像素灰度值，如果 $p$ 比 $q$ 大就取1，否则取0。 随机选取特征点附近的N个点， 几个点就会产生几个 0 或 1。
    



###  1.1.3 特征匹配 

特征匹配解决SLAM中的数据关联问题，即确定当前看到的路标与之前看到的路标之间的对应关系，通过对图像与图像或者图像与地图之间的描述子进行准确匹配，方便后续的姿态估计。



假设在 $t$ 时刻获得的图像 $I_t$ 上我们提取到了特征点 $x_t^m, \quad m=1,2,...,M$ ，而在 $t+1$时刻上我们提取了特征点 $x_{t+1}^n，\quad n=1,2,...N$，  匹配这两个集合元素的方法:

-  **暴力匹配** ，即对每一个特征点 $x^m_t$ 与所有的 $x_{t+1}^n$测量描述子的距离，然后排序，取最近的一个作为匹配点。对于浮点类型的描述子距离常用欧式距离，而对于BRIEF类似的描述子用汉明距离即可。
- **快速近似最近邻(FLANN)** 当特征点数量很大时，暴力匹配的运算量将会变得很大。







## 1.2. 2D-2D对极几何

### 1.2.1 对极约束

假设我们从两张图像中，得到了一对匹配好的特征点，如果有若干个匹配点，我们就可以通过这些二维图像点的对应关系，恢复两帧之间摄像机的运动。

![image-20220510180652364](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220510180652364.png)

如图我们想求取两图像 $I_1$ 和 $I_2$ 之间的运动 $R$ 与 $t$ ，两个相机中心为 $O_1$ 和 $O_2$ 。考虑$I_1$ 中的一点 $p_1$， 其在 $I_2$ 上的对应着 $p_2$ 。 点 $O_1, O_2, P$ 三个点可以确定一个平面称为极平面， $e_1, e_2$ 称为极点， $O_1, O_2$ 称为基线， 而 $l_1, l_2$ 称为极线。

从第一帧的角度看，射线 $\vec{O_1P_1}$ 是某个像素可能出现的空间位置，如果我们不知道 $P$ 的空间位置，则连线 $\vec{e_2P_2}$就是 $P$ 可能出现的投影位置，现在我们通过特征匹配求得了特征点的像素位置 $p_2$ ，我们就能判断 $P$ 的空间位置。



在第一帧的坐标系 (设为世界坐标系)下，设 $P$ 的 **空间位置** 为$P = [X, Y, Z]^T$ 

两个像素点 $p_1, p_2$ 的像素位置为：
$$
s_1p_1 = KP, \quad \quad s_2p_2 = K(RP + t)
$$
$K$ 为相机内参矩阵，如果用齐次坐标,因为齐次坐标下乘以非零常数没有意义我们可以写为:
$$
p_1 = KP, \quad \quad p_2 = K(RP + t)
$$
取 两个像素点 $p_1, p_2$在归一化平面上的坐标:
$$
x_1 = K^{-1}p_1 \quad \quad x_2 = K^{-1}p_2
$$
代入上式可得:
$$
x_2 = Rx_1+t
$$
同时左乘 $t^{\land}$, 即对 $t$做的 外积:
$$
t^{\land}x_2 = t^{\land}Rx_1
$$
然后两侧同时左乘 $x_2^T$
$$
x_2^Tt^{\land}x_2 = x_2^Tt^{\land}Rx_1
$$
因为 $t^{\land}$ 是一个与 $t, \quad x_2$垂直的向量，所以等式左边为0，可得：
$$
x_2^Tt^{\land}Rx_1 = 0
$$
重新带入 $p_1, p_2$有
$$
p_2^TK^{-T}t^{\land}RK^{-1}p_1 = 0
$$
该式称为 **对极约束** ，我们记 $E = t^{\land}R$ 为本质矩阵， $F = K^{-T}t^{\land}K^{-1}$为基础矩阵，则我们可以简化对极约束:
$$
x_2^TEx_1 = p_2^TFp_1 = 0
$$
所以相机位姿估计问题可以变为以上两步

- 1. 根据配对点的像素位置，求出$E$ 或者$F$
  2. 根据 $E$ 或者 $F$， 求出 $R, t$



### 1.2.2 本质矩阵

本质矩阵是一个 3×3的矩阵，内有9个未知数。

- 本质矩阵是由对极约束定义的，由于对极约束时等式为0的约束，所以对 $E$ 乘以任意非零常数后，对极约束仍然满足，所以 $E$ 在不同尺度下是等价的
- 根据矩阵 $E = t^{\land} R$ ,本质矩阵 $E$ 的奇异值(即 SVT中的$\sum$ ) 必定是 $[\sigma, \sigma, 0]^T$ 的形式，这是本质矩阵的 **内在性质**  （https://blog.csdn.net/zengxyuyu/article/details/106676509）

- 由于平移与旋转各有三个自由度， 故 $t^{\land}R$ 共有六个自由度, 但由于尺度等价性，E实际上有5个自由度。



因为他有5个自由度， 所以最少可以用五对点来求解 $E$， 但是 $E$ 的内在性质是一个非线性性质，在求解线性方程时会有麻烦，因此也可以只考虑他的尺度等价性，使用八对点来估计 $E$ 。

考虑一对匹配点，他们的归一化坐标为: $x_1 = [u_1, v_1, 1]^T, x_2 = [u_2, v_2, 1]^T$, 根据对极约束有:
$$
\left(
 \begin{matrix}
   u_1 & v_1 & 1\\
  \end{matrix} 
\right)\left(
 \begin{matrix}
   e_1 & e_2 & e_3 \\
   e_4 & e_5 & e_6 \\
   e_7 & e_8 & e_9 
  \end{matrix} 
\right)\left(
 \begin{matrix}
   u_2 \\ v_2 \\ 1\\
  \end{matrix} 
\right) = 0
$$
把矩阵$E$ 展开，写成向量的形式:
$$
e = [e_1, e_2, e_3, e_4,..., e_9]^T
$$
那么对于对极约束有:
$$
[u_1u_2, u_1v_2, u_1,v_1u_2,v_1v_2,v_1,u_2,v_2,1]e = 0
$$
![image-20220510210415204](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220510210415204.png)

$e$ 位于该8*9矩阵的零空间里，如果矩阵是满秩的8，则零空间维度为1，$e$构成一条线，这与$e$的尺度等价性是一致的，如果八对匹配点组成的矩阵满足秩等于8，那么 $E$ 的各元素就可以由上式解得。

现在已经求得 $E$, 来求相机运动 $R$ , $t$ 。我们知道  $\sum = diag(\sigma, \sigma, 0)$ ，在SVD分解中，对于任意一个 $E$ ，存在两个可能的 $t$ , $R$ 与它对应:
$$
t_1^{\land} = UR_Z(\frac{\pi}{2})\Sigma U^T, \quad R_1 = UR_Z^T(\frac{\pi}{2})V^T \\
t_2^{\land} = UR_Z(-\frac{\pi}{2})\Sigma U^T, \quad R_2 = UR_Z^T(-\frac{\pi}{2})V^T
$$
其中 $R_Z(\frac{\pi}{2})$ 表示沿 $Z$ 轴旋转90°得到的旋转矩阵，同时由于 $-E$ 和 $E$ 等价，所以对任意一个 $t$ 取负号，也会得到同样的结果。

![image-20220510231038088](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220510231038088.png)

在四个可能的解中只有一个 $P $ 的深度是正的，因此只要把任意一点代入四种解中，检验该点在两个相机的深度，就可确定正确的解。



剩下的问题还有一个，根据线性方程结出来的 $E$ 可能不满足 $E$ 的内在性质，它的奇异值不一定满足 $\sigma, \sigma, 0$ 的形式，通常的做法是在对8点法的 $E$ 进行SVD分解后，会得到奇异值矩阵 $\Sigma = diag(\sigma_1,\sigma_2, \sigma_3)$， 设 $\sigma_1 > \sigma_2>\sigma_3$, 将求出来的矩阵投影到 $E$ 所在的流形上， 即:
$$
E = Udiag(\frac{\sigma_1 + \sigma_2}{2}, \frac{\sigma_1 + \sigma_2}{2}, 0)V^T
$$
更简单的做法也有直接将奇异值矩阵取成 $diag(1,1,0)$



8点法的讨论:

- 单目SLAM难以初始化， （即一开始只知道两个特征点的两个像素位置，不知道特征点位置，难以使用）
- 尺度不确定性(因为 $E$ 的内在特性)，因为$R$ 是正交矩阵，我们常通过限制 $t$ 的方式来求取合理的解
- 纯旋转问题: $t=0$ 时无法求解
- 多于8个匹配点时，超定问题，常用最小二乘法求解



### 1.2.3 三角测量

通过在两处观察同一个点的夹角，确定该点的距离。

根据对极几何中的定义，设$x_1, x_2$ 为两个特征点的归一化坐标，则他们满足:
$$
s_1x_1 = s_2Rx_2 + t
$$
 我们左乘 $x_1^{\land}$ 得:
$$
s_1x_1^{\land}x_1 = 0 = s_2x_1^{\land}Rx_2 + x_1^{\land}t
$$
即可求得深度 $s_2$, $s_1$ 同理



