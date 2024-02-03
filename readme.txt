项目实现无人机避障制导技术
prop_guidance实现无人机制导律的汇总，应包括比例制导律，与避障制导律；   /////////////50%
drone_trajs 包含了无人机用到的轨迹算法，包括多项式轨迹，B样条曲线；           /////////////////////////100%
target_observer 实现基于视觉的目标位置估计与速度估算；                                        //0%

接下来要做的是：

1.      完善避障制导律理论及代码；
2.      制导律得到的轨迹转换成多项式轨迹；进行minimum snap计算，作为全局轨迹；
3.      考虑目标在一个范围内变动机动模式，或者由于速度观测，位置观测的噪声影响时，所换
          算多项式轨迹的动力学约束不满足切换后实时的飞行，因此对初始段某个范围作优化，将
          该段轨迹推向上一次采样获取的轨迹;     可能需要转化成B样条全局优化；
4.      回顾B样条曲线，思考在该工程中的作用；
5.      写目标状态估计的功能包，包括位置估计与速度估计； 




sudo apt update  
sudo apt install git

git config --global user.name "Your Name"  
git config --global user.email "your-email@example.com"

mkdir myproject  
cd myproject

git init

git add .

git commit -m "Initial commit"

git remote add origin https://github.com/your-username/repository-name.git

git push -u origin master






每次上传时：
git add .
git commit -m "Initial commit"
git push -u origin master

DuHongbao
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
