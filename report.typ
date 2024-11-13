#set text(font: "Times New Roman", size: 14pt)

= 可视计算与交互概论 Lab 1

== Task 2: Spring-Mass Mesh Parameterization(1.5')

网格的参数化。

使用讲义上给的方法。

$ E = sum_i sum_j 1/2 D_(i,j) norm(t_i - t_j) $

求偏导为0,得

$ t_i = sum_j 1/2 D_(i,j) t_j $
记$lambda_(i,j) = D_(i,j) / (sum_j D_(i,j))$,方程化简为
$ t_i - sum_j lambda_(i,j) t_j = 0 $

即$ A vec(t_1,t_2,dots.v,t_n) = 0 $

其中$ A = mat(lambda_(i.j)) $

上述方程容易使用Gauss-Seidel方法求解。现在需要选择$lambda$的取值。

+ 从$D_(i,j)$入手，将$D_(i,j)$设置为两点在三维空间中的距离倒数。相当于弹簧的劲度系数和长度成正比。
+ $lambda = 1/n$，$n$是其邻居的个数

边缘点的坐标为了方便起见，初始化到了一个半径为$1$的圆上。

结果如下。
#figure(image("pic/mesh_para_vanilla.png"), caption: [$D_(i,j) = 1/l$效果])

#figure(image("pic/mesh_para_uni.png"), caption: [$lambda = 1/n$效果])

== Task 3: Mesh Simplification(2.5')

按照讲义的公式，计算二次代价，计算塌陷后点坐标$v=Q^(-1) vec(0,0,0,1)$和塌陷代价$v^T Q v$.循环塌陷可行最小代价边，更新和这两个点有关的代价。

对于不可逆的情况，判断$det(Q)$，若绝对值小于阈值，计算坐标使用两端点和中点三点中代价最小的点。

#figure(image("pic/simplification_block_3.png"), caption: [Mesh Simplification])

#figure(image("pic/simplification_rocket_4.png"), caption: [Mesh Simplification])

#figure(image("pic/simplification_sphere_3.png"), caption: [Mesh Simplification])

== Task 4: Mesh Smoothing (1') 

和要求给的公式略有区别，我使用ppt上给的公式，即$v := v + lambda Delta v$,其中$Delta v$为mesh的拉普拉斯算子。分别实现 uniform laplacian 和 cotangent laplacian .流程与ppt一致。实验时容易发现， cotangent laplacian 的数值稳定性很差，需要做裁剪。将取值裁剪到 $(-1000,1000)$ 即可。

#figure(image("pic/smooth_block_uni.png"), caption: [uniform laplacian])

#figure(image("pic/smooth_block_cot.png"), caption: [cotangent laplacian])

由于裁剪，cot表现力大幅下降，其平滑效果与 uniform laplacian 差异不大。

== Task 5: Marching Cubes (2') 

枚举每一个block,将其8个点带入sdf计算，如果小于0，那么该点在内部，否则在外部。

== Code Availability

报告中所有图片均在 `pic` 文件夹中，并附加了 `tasks.cpp` 文件，其包含了上述所有内容的实现代码。部分代码已经被注释。