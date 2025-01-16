package core

var DecideTime int = 10 //决策时间间隔

var DecideCount int = 18 //决策次数
// const NeedChargePower float64 = 15 //能移动的距离低于多少需要充电
const ZeroEpsilon float64 = 1e-10
const alpha = 1

//1/(2,8)   10
//1 3 ->8 1 3
//1/(2,9)   10
//           10
