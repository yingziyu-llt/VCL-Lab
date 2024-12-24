= lab 5 实验报告

== Task 1 平行坐标

利用proxy类的各种性质、提供的各种接口，获得鼠标位置、鼠标点击事件、鼠标拖动事件，分别激活不同的事件处理函数，实现鼠标的拖动、点击操作所对应的更改对应区间、更改焦点的功能。

平行坐标只需要按照不同的特征，在两坐标轴对应点之间连线即可。

本次实验中，我们实现了平行坐标的绘制，以及鼠标点击、拖动事件的处理，相对来说较好的模仿了样例中的效果，见下。

#figure(image("pic/parallel_1.png",height: 40%),caption: "平行坐标图")
#figure(image("pic/parallel_2.png", height: 40%),caption: "选中部分元素")
#figure(image("pic/parallel_3.png",height: 40%),caption: "按其他坐标轴排序")
#figure(image("pic/parallel_4.png",height: 40%),caption: "多重筛选")
#figure(image("pic/parallel_5.png",height: 40%),caption: "点击某坐标轴复原")

== Task 2 LIC算法

与给出的教程类似，我使用LIC算法进行流体场的可视化。从每个点出发，向其流场正方向前进，将其路径上的点对应的背景噪声和kernel function相乘再相加（即所谓卷积）后，得到一个值，根据这个值来决定该点的颜色。