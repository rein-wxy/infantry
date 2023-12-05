# infantry
# RM 步兵控制

- [RM 步兵控制](#rm-步兵控制)
  - [摩擦轮控制弹速](#摩擦轮控制弹速)

## 摩擦轮控制弹速

摩擦轮电机采用3508可实现电机速度闭环
实测摩擦轮温度会很大程度上影响射速

- 发弹量上去后摩擦轮升温

故摩擦轮电机目标值不能为定值需根据裁判反馈实时弹速做出改变

~~~ c
/*
 speed:预期目标值
	limit:限制速度15 18 30
 将系数映射到tan正负二分之Π之间
  return  目标改变量
*/

float speed_offest(float speed,float limit)
{
	float a = (speed - limit),b;
	if(a<=0.5 && a >=-0.5)  	 
		b =tan( a*1.571);	//pai/2
	else if	(a>=0.5 )		 
		b = tan(0.785);			//限制x防止弹速变化大
	else if	(a<=-0.5 )    
		b = tan(-0.785) ; 
	return b* 10;
}
~~~

**实测上述方式可实现弹速0.5m/s内波动**
