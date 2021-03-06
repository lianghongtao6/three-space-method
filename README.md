# 三空间划分模型

用于在集装箱等大型存储仓库中放入物品


## 算法思路

在集装箱中只需要装载一个货物的情况下，我们首先对需要装载的货物进行筛选，其基本思想是放入空间利用率最高的货物。前文中我们已经假设货物不需只能正面朝上放置，所以我们先将放置货物空间的边长和货物的边长进行比较，如果货物的最长边和最短边都小于装载空间的最长边和最短边，则表示该货物可以放入。我们使用贪心算法，轮询并筛选出空 间利用率最大的可以放入该空间的货物。 货物放入该空间后，如上图所示，剩余空间被划分为三个空间，利用递归的思想，将每一个空间作为一个新的空间，筛选并放入空间利用率最大的货物。空间划分的过程继续进行， 直到剩余空间不能放入任何现有的货物。这样，我们就可以遍历所有的情况，并找到一个装载货物的最优方案。

2019年美赛B题练习程序
