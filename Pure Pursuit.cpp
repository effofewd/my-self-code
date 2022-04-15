#include<stdio.h>
#include<math.h>
#include<string.h>
struct AutoCar
{
	float x;
	float y;
	float yaw;
	float  v;
};
#define k  0.1  // 前视距离系数
#define Lfc  2.0 // # 前视距离
#define Kp  1.0  //# 速度P控制器系数
#define dt  0.1  //# 时间间隔，单位：s
#define L  2.9  //# 车辆轴距，单位：m
#define target_speed 100.0 / 3.6 // 速度 ，单位:km/h 
#define T 100.0  //时间 ，单位：s 
int calc_target_index(struct AutoCar *p);
int pure_pursuit_control(struct AutoCar *p,int pind); 
static float cx[63] = {300.0    ,     298.50124958 , 294.01997335 , 286.60094674 , 276.3182982,
      263.27476857 , 247.60068447,  229.45265619  ,209.0120128 ,  186.48299048,
      162.09069176 , 136.07883643 , 108.70732634 ,  80.24964859 ,  50.99014287,
       21.2211605 ,  -8.75985669 , -38.65334829 , -68.16062841 , -96.98687006,
     -124.84405096 ,-151.45383138 ,-176.55033518, -199.88280638, -221.21811466,
     -240.34308466 ,-257.06662601 ,-271.22164261, -282.6667022  ,-291.28744954,
     -296.99774898, -299.74054508 ,-299.48843274, -296.24393097 ,-290.03945777,
     -280.93700619 ,-269.0275249 , -254.43000951, -237.29031357 ,-217.77969126,
     -196.09308626, -172.44718396 ,-147.0782464  ,-120.23975162 , -92.19986099,
      -63.23873983 , -33.64575808 ,  -3.71659904 ,  26.24969503 ,  55.95371083,
       85.09865564,  113.39332281 , 140.55500139,  166.31230085 , 190.40786278,
      212.60093229,  232.66976355 , 250.41383545  ,265.65585508 , 278.24352922,
      288.051086  ,  294.98053153,  298.96262911},
    cy[63] = { 0.0 ,          19.96668333 ,  39.73386616 ,  59.10404133,   77.88366846,
       95.88510772 , 112.92849468 , 128.84353745 , 143.47121818 , 156.66538193,
      168.29419696 , 178.24147201 , 186.40781719 , 192.71163708 , 197.089946,
      199.49899732 , 199.91472061 , 198.33296209,  194.76952618,  189.26001754,
      181.85948537 , 172.64187333 , 161.69928076 , 149.14104244 , 135.09263611,
      119.69442882 , 103.10027436 ,  85.47597605 ,  66.99763003 ,  47.84986584,
       28.22400161 ,   8.31613249 , -11.67482869 , -31.54913883 , -51.10822041,
      -70.15664554 , -88.50408866 ,-105.96722818 ,-122.37157819 ,-137.55323184,
     -151.36049906 ,-163.65542221, -174.31515448 ,-183.23318735 ,-190.32041478,
     -195.50602353 ,-198.73820073 ,-199.98465151 ,-199.23292177 ,-196.49052252,
     -191.78485493 ,-185.16293647, -176.69093114 ,-166.45348844 ,-154.55289751,
     -141.10806511 ,-126.25332757, -110.13710852 , -92.92043588 , -74.77533297,
      -55.88309964 , -36.43250085 , -16.61788056};   //跟踪坐标 
static struct AutoCar Car={300.0,0,0,0};  //初始化车辆初始状态 
int main()
{   int lastIndex,target_ind,i=0;
	float time=0.0;
	lastIndex=sizeof(cx)/4-1;
	target_ind=calc_target_index(&Car);
	while(T>=time&&lastIndex > target_ind)
	{	
       
        target_ind = pure_pursuit_control(&Car,target_ind);
       
        time = time + dt;
       
       printf("%f  x=:%f %f  %d \n",Car.x,Car.y,Car.v,target_ind) ;
	} 
	return 0;
}
int pure_pursuit_control(struct AutoCar *p,int pind)
{
	int ind;
	float tx,ty,alpha,Lf,ai,delta;
	
	ai = Kp*(target_speed-p->v);
	
    ind = calc_target_index(&Car);

    if (pind >= ind)
        ind = pind;

    if (ind < sizeof(cx)/4)
    { 
		tx = cx[ind];
	    ty = cy[ind];
	}
    else 
	{ 
		ind = sizeof(cx)/4 - 1;
	    tx = cx[ind];
        ty = cy[ind]  ;
	}  
    alpha =atan2(ty - p->y, tx - p->x) - p->yaw;

    if (p->v < 0) // # back
        alpha = M_PI - alpha;

    Lf = k * p->v + Lfc;

    delta =atan2(2.0 * L * sin(alpha) / Lf, 1.0);  // 前轮转角 
    // 更新车辆状态 （x坐标、y坐标、偏转角、速度） 
	p->x = p->x + p->v * cos(p->yaw) * dt;
    p->y = p->y + p->v * sin(p->yaw) * dt;
    p->yaw = p->yaw + p->v / L * tan(delta) * dt;
    p->v = p->v + ai * dt;
    
    return  ind ;
}
//  函数用于搜索最临近的路点：
int calc_target_index(struct AutoCar*p)
{	float d[63]={0},min,Ls,Lf;
	int min_index=0;
	float DX,DY;
	for(int i=0;i<sizeof(cx)/4;i++)
	  d[i]=abs(sqrt((p->x-cx[i])*(p->x-cx[i])+(p->y-cy[i])*(p->y-cy[i])));	
	min=d[0];
	for(int j=1;j<sizeof(d)/4;j++)
		if(min>d[j]) 
		{
		min_index=j;
		min=d[j];
		}
    Ls = 0.0;
    Lf = k * p->v + Lfc;
    while( Lf > Ls && (min_index + 1) < sizeof(cx)/4)
    {
        DX = cx[min_index + 1] - cx[min_index];
        DY = cy[min_index + 1] - cy[min_index];
        Ls += sqrt(DX*DX +DY*DY );
        min_index += 1;
    }
    return min_index;
}
