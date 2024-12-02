#set text(font: "Times New Roman")
= 第四次作业报告
林乐天 2300012154

== Task 1: Phong Illumination (1.5', bonus=1.5') 
根据 Phong shading 模型，每个点的亮度应该为$ k_d (I_a + I_d max(0,n dot l)) + k_s I_d max(0,r dot v)^p $,其中$n$为法向量方向，$l$为光照方向，$r$为反射光线方向，$v$为视线方向。满足$r = 2(n dot l) n - l$.直接使用公式计算即可。

对于 Blinn-Phong 反射模型,只需要将反射项修改为$k_s I_d max(0,h dot n)^p$，其中$h = (v + l) / 2$.
#figure(image("pic/phone_floor.png"),caption:[phone shading])
#figure(image("pic/blinn-phone_floor.png"),caption:[blinn-phone shading])

回答问题：
=== 顶点着色器和片段着色器

一般来说，顶点着色器将顶点进行处理，传入后续流程，片段着色器利用顶点着色器处理得到的信息，对像素进行染色等工作。二者通过`layout(location = 0) in vec3 v_Position;
`等输入输出变量进行数据传递，或通过location和相同变量名等建立绑定变量，从而进行信息传递。

=== `if (diffuseFactor.a < .2) discard;`

这个语句是为了进行透明度裁剪，削去部分透明面片，让模型更加自然。如果将其改为==0.0，一主要问题是计算精度问题。由于浮点误差等原因，==0.00会使得部分不应该被保留的面片被错误保留，造成图像的失真。实验如下。
#figure(image("pic/phong_whiteoak.png"),caption: [不修改阈值])
#figure(image("pic/bad_whiteoak.png"),caption: [修改为==0.0])

不少透明的树叶产生了反光，造成了失真现象。

=== bonus

参考#link("https://gamedev.stackexchange.com/questions/174642/how-to-write-a-shader-that-only-uses-a-bump-map-without-a-normal-map",[ StackExchange 资料])，高度贴图主要任务是根据高度修改该点的法向量。计算其屏幕空间中的梯度，利用高度图的采样结果计算表面纹理梯度，混合生成凹凸法线，最终在不改变几何形状的前提下重现了凹凸表面。效果见@fig5

#figure(image("pic/phone_sibenik.png"),caption: [凹凸表面])<fig5>

== Task 2: Environment Mapping (1') 

首先根据文档将天空盒的坐标设置为 ` (u_Projection * u_View * vec4(a_Position , 1.0)).xyww`.启动，渲染不正常。放大后发现天空盒在模型内部，是一个小盒子。为了让其覆盖整个场景，将`a_Position`设置为10000倍。天空盒即可显示正常。

对于环境映射，根据文档计算从环境中得到的颜色为`texture(u_EnvironmentMap, reflect(-viewDir, normal)).rgb * u_EnvironmentScale`，将其累加到最终的颜色上即可。

#figure(table(columns:2,[#image("pic/environment_teapot.png")],[#image("pic/environment_bunny.png")]),kind:image,caption: "环境映射")

== Task 3: Non-Photorealistic Rendering (1.5')

根据讲义公式，实际上就是认为物体为朗伯体，并将亮度转化为一个冷暖色之间的插值。为了让图像和讲义上的完全一样，即产生所谓“卡通效果”，我选择对其颜色做分段。选择的方法是将其插值的参数乘2，向下取整，再除以2，即将其分为`0,0.5,1`三段。得到了如下效果图

#figure(table(columns:2,[#image("pic/npr-teapot.png")],[#image("pic/npr-bunny.png")]),kind:image,caption: "非真实渲染")

问题：
=== 如何分别渲染模型的反面和正面的
在渲染背面时，使用```cpp
glCullFace(GL_FRONT);
glEnable(GL_CULL_FACE);
```
方法，剔除了正面，从而只渲染反面。正面类似。
=== 轮廓线渲染
+ 轮廓线的形状不一定可以用法向量来描述，故按法向量扩展产生的线条可能不能很好的描绘轮廓线的真实形状。
+ 由于边界实际上是一系列三角形，按照法向量扩展会造成边界的不连续，影响感官。
+ 没有充分考虑视角变换，视角旋转的时候过渡不够流畅。
== Task 4: Shadow Mapping (1.5') 

通过直接从shadow map中找到对应点的深度值，比较本点的depth值，就可以得到是否被shadow的信息。

问题：
=== 投影矩阵
有向光源是平行光，应当用正交投影矩阵。点光源最符合透视投影矩阵。
=== 为什么不用计算深度
深度值计算是 OpenGL 的固定功能管线的一部分，由投影矩阵、视图矩阵和模型矩阵的变换过程自动生成。
#figure(image("pic/shadowmapping_whiteoak.png"),caption: "阴影映射")
== Task 5: Whitted-Style Ray Tracing (2', bonus=2.5') 

=== 光线求交算法

利用三角面片求交算法(公式见notes 15.10)，求出每个面片和光线的交点坐标，再根据交点坐标计算光线的颜色。

=== Whitted-style ray tracing
对于每个面片，计算其法向量，光线与面片的交点，再根据Phong模型计算颜色。

=== Shadow ray
检测光线与光源之间是否有遮挡物，若有则不进行光照计算。

#figure(table(columns:2,[#image("pic/raytraacing_cornell.png")],[#image("pic/raytraacing_cornell_noshadow.png")]),kind:image,caption: "光线追踪渲染结果（有无阴影）")

问题：
==== 光线追踪和光栅化的渲染结果有何异同

光线追踪算法恰当地显示了左右两面墙反射到物体上造成的颜色变化和多次反射的效果，而光栅化没有做出这个效果，只是简单地显示了物体的颜色。这是因为光线追踪算法是基于物理光线的传播，而光栅化是基于像素的颜色填充。光线追踪算法的计算量较大，但可以得到更加真实的效果。

#figure(table(columns:2,[#image("pic/raytraacing_cornell.png")],[#image("pic/shadowmapping_cornell.png")]),kind:image,caption: "光线追踪和光栅化的渲染结果")

=== bonus

使用KDTree加速面片求交速度，架构已经呈现在代码中，没调通。