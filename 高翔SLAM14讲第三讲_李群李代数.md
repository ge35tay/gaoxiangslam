# 高翔SLAM14讲 第三讲



SLAM中位姿是未知的，我们需要解决什么样的相机位姿符合当前观测数据的问题， 典型的方法：把它构建成一个优化问题，求解最优的$R ,T$ ，使得误差最小化。但是由于旋转矩阵自身的限制（行列式为1的正交矩阵），我们希望姿态估计变成一个无约束的优化问题。

## 1. 李群李代数基础

三维旋转矩阵 ==> 特殊正交群 $SO(3)$, 变换矩阵 ==>特殊欧式群 $SE(3)$
$$
SO(3) = \{ R \in R^{3×3} | RR^T = I, det(R) = 1\}
$$

$$
SE(3) = \{T = \left[\begin{array}{cccc} R  &  t \\ 0^T & 1 \end{array} \right] \in R^{4×4} | R \in SO(3), t \in R ^ 3 \}
$$

可以注意到， 三维旋转矩阵和变换矩阵对加法都不是封闭的。 但对于乘法，他们都是封闭的。
$$
R_1 R_2 \in SO(3); T_1T_2 \in SE(3)
$$
对于这种只对一个运算封闭的集合，我们称之为 **群**

### 1.1 群

**群是一种集合加上一种运算的代数结构** 我们把集合记作 $A$, 运算记作 $^.$,  则群可以记为 $G = (A, ^.)$. 对于群它需要满足:

- **封闭性**： $ \forall a_1, a_2 \in A, a_1 ^. a_2 \in A$
- **结合律**: $\forall a_1, a_2, a_3 \in A, (a_1^.a_2)^.a_3 = a_1^.(a_2^.a_3)$

- **幺元（单位元）**：  $\exists a_0 \in A, s.t. \forall a \in A, a_0 . a = a . a_0 = a$
- **逆**： $\forall a \in A, \exists a^{-1}\in A, s.t. a.a^{-1} = a_0$  (A中的任意元素都有逆而且其与逆相乘能得到单位元)

可以知道三维旋转矩阵和矩阵乘法 (==旋转矩阵群==)以及变换矩阵和矩阵乘法(==变换矩阵群==)都构成群

 

> 李群是指具有连续（光滑）性质的群， $SO(n)$ 与 $SE(n)$ 在实数空间上都是连续的， 直观上想象一个刚体在空间内可以连续运动，所以都是李群



### 1.2 李代数

对于旋转矩阵有:
$$
RR^T = I
$$
设旋转矩阵随时间变化， 对等式时间两边求导
$$
R(t)\dot R(t)^T  + \dot R(t) R(t)^T = 0 \\
\dot R(t) R(t)^T = -R(t)\dot R(t)^T
$$
所以 $\dot R(t) R(t)^T$ 是一个反对称矩阵，联想到叉积可以表示成一个反对称矩阵($a^{\land}$)与向量的乘法， 对于 $\dot R(t) R(t)^T$  我们也可以找到一个与之对应的向量 $\phi$, 使得:
$$
\dot R(t) R(t)^T = \phi ^ {\land} \\
\dot R(t) R(t)^T \vee = \phi
$$


右乘$R(t)^T$可得
$$
\dot R(t) = \phi ^{\land} R(t) = \left[\begin{array}{cccc} 0 & -\phi_3  & \phi_2  \\ \phi_3 & 0 & -\phi_1 \\ -\phi_2 & \phi_1 & 0 \end{array} \right]R(t)
$$
对旋转矩阵取一次导数，只需左乘一个 $\phi ^{\land} (t)$ 矩阵即可， $\phi$ 反映了 $R$的导数性质，所以他在$R$ 的正切空间上。



### 1.3 李代数的定义

 每个李群都有与之对应的李代数， 李代数描述了李群的局部性质。

**李代数由一个集合 $\mathbb{V}$(三维矩阵),一个数域$\mathbb{F}$ (常数)和一个二元运算$[,]$ 组成**，若他们满足以下条件，则称 \{$\mathbb{V, F}, [,]$ \}为一个李代数， 记作 $\mathfrak{g}$

- **封闭性**  $\forall X, Y \in \mathbb{V}, [X, Y] \in \mathbb{V}$

- **双线性**  $\forall X, Y, Z \in \mathbb{V}, a, b \in \mathbb{F}$ 有
  $$
  [aX + bY, Z] = a[X, Z] + b[Y, Z], [Z, aX + bY] = a[Z, X] + b[Z, Y]
  $$

- **自反性** $ \forall X \in \mathbb{V}, [X,X] = 0$

- **雅可比等价**  $\forall X, Y, Z \in \mathbb{V}, [X, [Y, Z]] + [Z, [Y, X]] + [Y, [Z, X]] = 0$

二元计算$[,]$ 被称为李括号 (Lie bracket), 他表达了相乘两个元素的差异性(自反性，与自己相乘为0) 

e.g.

三维向量$\mathbb{R}^3$上定义的叉积是一种李括号， 而 $\mathfrak{g} = (\mathbb{R}^3, \mathbb{R}, ×)$ 构成了一个李代数



### 1.4 李代数 $\mathfrak{so}(3)$

$SO(3)$李群对应的李代数是定义在$\mathbb{R} ^3 $上的三维向量， 记作 $\phi$， 设$\phi $ 的反对称矩阵为 $\Phi$, 则两个向量 $\phi_1, \phi_2$ 的李括号为:
$$
s[\phi_1, \phi_2] = (\Phi_1\Phi_2 - \Phi_2\Phi_1)^{\vee}
$$
所以李代数 $\mathfrak{so}(3)$ 的元素是3维向量或者三维反对称矩阵,
$$
\mathfrak{so}(3) = {\phi \in \mathbb{R}^3, \Phi = \phi^{\land} \in \mathbb{R}^{3×3}}
$$


他与李群 $SO(3)$的关系由指数映射指定:
$$
R = exp(\phi^{\land})
$$

### 1.5 李代数 $\mathfrak{se}(3)$

$\mathfrak{se}(3)$ 位于 $\mathbb{R}^6$ 空间中:
$$
\mathfrak{se}(3) = \{ \xi = \left[\begin{array}{cccc} \rho  \\ \phi \end{array} \right] \in \mathbb{R}^6, \rho \in \mathbb{R}^3, \phi \in \mathfrak{so}(3), \xi^{\land} = \left[\begin{array}{cccc} \phi^{\land} & \rho \\ 0^T & 0 \end{array} \right] \in \mathbb{R}^{4×4} \}
$$
每个 $\mathfrak{se}(3)$ 李代数元素记作 $ \xi$ , 他是一个六维向量，前三维为平移 $\rho$,  后三维为旋转 $\phi$ ($\mathfrak{so}(3)$)



> $\mathfrak{se}(3)$ 可以理解为由一个平移加上一个$\mathfrak{so}(3)$ 元素构成的向量，虽然 $\rho$ 还不直接是平移

李括号:
$$
[\xi_1, \xi_2] = (\xi_1^{\land}\xi_2^{\land} - \xi_2^{\land}\xi_1^{\land})^{\vee}
$$




## 2. 指数与对数映射

设$t_0$ = 0, 并设此时旋转矩阵为 $R(0) = I$ . 对旋转矩阵 $R(t)$ 在 0附近泰勒展开
$$
R(t) ≈ R(t_0) + \dot R(t_0)(t - t_0) \\
= I + \phi(t_0)^{\land}(t)
$$
设 $\phi$ 保持为常数 $\phi(t_0) = \phi_0$, 则
$$
\dot R(t) = \phi(t_0)^{\land}R(t) = \phi_0 ^ {\land}R(t)
$$
结合$R$的初始值可解得:
$$
R(t) = exp(\phi_0 ^ {\land}t)
$$

该矩阵指数的计算?                                                  


### 2.1 $SO(3)$上的指数映射

**任意矩阵的指数映射可以写成一个泰勒展开**,  但是只有在收敛的情况下才会有结果，其结果仍是一个矩阵:
$$
exp(A) = \sum_{n=0}^\infty \frac{1}{n!}A^n
$$
同样对于$\mathfrak{so}(3)$的任一三维向量元素 $\phi$, 亦可定义它的指数映射:
$$
exp(\phi^{\land}) = \sum_{n=0}^\infty \frac{1}{n!}(\phi^{\land})^n
$$
假设三维向量$\phi$的模长和单位方向分别为 $\theta$ 和 $a$, 对于 $a^{\land}$ 有
$$
a^{\land}a^{\land} = aa^T - I \\
a^{\land}a^{\land}a^{\land} = -a^{\land}
$$
利用这两项我们可以处理指数的高阶项	

![image-20220501113231081](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220501113231081.png)

罗德里格斯公式！！！把 $\theta$ 当成角度模， $a$ 当成旋转方向



所以 $\mathfrak{so}(3)$ 是旋转向量组成的空间，而其通过指数映射映射到李群$SO(3)$ 上



反过来的对数映射，我们也可以从李群$SO(3)$ 映射到$\mathfrak{so}(3)$ 上:
$$
\phi = In(R)^{\vee} = ( \sum_{n=0}^\infty \frac{(-1)^n}{n+1}(R-I)^{n+1})^{\vee}
$$
不过我们一般还是通过下式计算旋转矩阵对应的李代数
$$
\theta = arccos(\frac{tr(R) - 1}{2})
$$

### 2.2 $SE(3)$上的指数映射

$$
\mathfrak{se}(3) = \{ \xi = \left[\begin{array}{cccc} \rho  \\ \phi \end{array} \right] \in \mathbb{R}^6, \rho \in \mathbb{R}^3, \phi \in \mathfrak{so}(3), \xi^{\land} = \left[\begin{array}{cccc} \phi^{\land} & \rho \\ 0^T & 0 \end{array} \right] \in \mathbb{R}^{4×4} \}
$$

$\mathfrak{se}(3)$ 上的指数映射：
$$
exp(\xi^{\land}) =  \left[\begin{array}{cccc} \sum_{n=0}^\infty \frac{(1}{n!}(\phi^{\land})^{n} & \sum_{n=0}^\infty \frac{1}{(n+1)!}(\phi^{\land})^{n}\rho   \\ 0^T & 1 \end{array} \right] \\
= \left[\begin{array}{cccc} R & J\rho   \\ 0^T & 1 \end{array} \right]
$$
其中, $J$ 可整理为
$$
J = \frac{sin\theta}{\theta}I + (1 - \frac{sin\theta}{\theta})aa^T+ \frac{1 - cos\theta}{\theta}a^{\land}
$$
所以RT矩阵中:
$$
t = J\rho
$$

## 3.李代数求导与扰动模型

### 3.1 BCH 公式与近似形式

当我们在 $SO(3)$中完成两个矩阵乘法时(即连续两个旋转)，李代数$\mathfrak{so}(3)$ 上发生了什么改变？

当$\mathfrak{so}(3)$ 上两个李代数做加法时(连续两个旋转向量)， $SO(3)$会发生什么样的变化？

若 $\phi_1$， $\phi_2$ 是标量，则有：
$$
exp(\phi_1^{\land})exp(\phi_2^{\land}) = exp((\phi_1 + \phi_2)^\land)
$$
很可惜在李代数中(旋转向量) 是一个三维向量，对应的**矩阵指数函数**也不是这样的展开.



> 根据**Baker-Campbell-Hausdorff** BCH 定理， 对于一个对应李群的李代数X,Y， 我们有:
> $$
> e^Xe^Y = e^Z \\
> Z = X + Y + \frac{1}{2}[A, B] + \frac{1}{12}[A, [A,B]] - \frac{1}{12}[B, [A,B]] + ...
> $$
> 其中 []是李括号, Lie bracket
>
> https://zhuanlan.zhihu.com/p/101212195

所以当处理两个矩阵指数之积时会产生一些由李括号组成的余项。 

进而， 在考虑$SO(3)$上的李代数 $in(exp(\phi_1^{\land})exp(\phi_2^{\land}))^{\vee}$ ,当$\phi_1$ 或$\phi_2$ 为小量时， 小量二次以上的项都可以被忽略，此时BCH有近似线性的表达:
$$
in(exp(\phi_1^{\land})exp(\phi_2^{\land}))^{\vee} ≈ \left\{
\begin{aligned}
J_l(\phi_2)^{-1}\phi_1 + \phi_2 & & if \ \phi_1 \  is \ small \\
J_r(\phi_1)^{-1}\phi_2 + \phi_1 & & if \ \phi_2 \  is \ small
\end{aligned}
\right.
$$
直观理解: 对1式，当旋转矩阵$R_2$ 左乘一个微小旋转矩阵 $R_1$ （李代数是 $\phi_1$）时，可以近似看作在原有李代数$\phi_2$ 的基础上加上了一个 $J_l(\phi_2)^{-1}\phi_1$ , 2式同理。 即李代数在BCH线性近似的条件下有左乘近似和右乘近似两种情况。

对于 **左乘**， $J_l$ 即为
$$
J_l = \frac{sin\theta}{\theta}I + (1 - \frac{sin\theta}{\theta})aa^T+ \frac{1 - cos\theta}{\theta}a^{\land}
$$
它的逆是:
$$
J_l^{-l} = \frac{\theta}{2}cot\frac{\theta}{2}I + (1 -\frac{\theta}{2}cot\frac{\theta}{2}) aa^T - \frac{\theta}{2}a^{\land}
$$
而对于**右乘** ,我们取旋转角的相反数即可:
$$
J_r(\theta) = J_l(-\theta) 
$$
 

e.g

> 假定对于某个旋转$R$, 其对应的李代数为 $\phi$, 左乘一个微小旋转 $\Delta R$,  则在李群上是为 $\Delta R ^{.}R$, 而在李代数上有:
> $$
> exp(\Delta \phi^{\land})exp(\phi^{\land}) = exp((\phi + J_l^{-1}(\phi)\Delta \phi)^{\land})
> $$
> 相反在李代数进行加法，让一个 $\phi$ 加上 $\Delta \phi$，可近似为李群带上左右雅可比的乘法:
> $$
> exp((\phi + \Delta \phi)^{\land}) = exp(\phi^{\land})exp((J_r \Delta \phi)^{\land}) = exp((J_l \Delta \phi)^{\land})exp(\phi^{\land})
> $$





对于$SE(3)$ 我们也有类似的BCH近似公式:
$$
exp(\Delta \xi^{\land})exp(\xi^{\land}) = exp((\mathcal{J}_l^{-1} \Delta \xi + \xi)^{\land}) \\
exp(\xi^{\land})exp(\Delta \xi^{\land}) = exp((\mathcal{J}_r^{-1} \Delta \xi + \xi)^{\land})
$$
其中 $\mathcal{J}_l$ 是一个 6×6的矩阵



### 3.2 $SO(3)$ 李代数的求导

现在考虑SLAM过程中对相机位置和姿态的估计，该位姿可以用$SO(3)$ 上的旋转矩阵或者 $SE(3)$ 上的变换矩阵来描述。我们假设机器人在位姿 $T$ 观察到一个世界坐标为 $p$ 的点，产生了一个观测数据 $z$, 由坐标变换关系可知:
$$
z = Tp + w
$$
$w$ 是噪声，则误差可以表示为:
$$
e = z - Tp
$$
对于机器人位姿的估计，相当于是寻找一个最优的机器人位姿 $T$ ,使得整体误差最小化
$$
min_TJ(T) = \sum_{i = 1}^{N} ||z_i - Tp_i||_2^2
$$
由李代数解决导数问题，则有两种思路, 1. 用李代数表示姿态，然后根据李代数加法来对李代数进行求导

2. 对李群左乘或右乘微小扰动，然后对该扰动求导，称为左扰动或右扰动模型。



![image-20220501173441223](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220501173441223.png)





![image-20220501173514087](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220501173514087.png)





![image-20220501173600641](C:\Users\24527\AppData\Roaming\Typora\typora-user-images\image-20220501173600641.png)
