package core

type NodeType int

const (
	DroneIdle             NodeType = iota // <u1>
	CarIdle                               // <car1>
	WorkerIdle                            // <w1>
	DroneToTask                           // <u1, t1>
	WorkerToTask                          // <w1, t1>
	DroneWorkerToTask                     // <u1, t1, w1>
	DroneToChargePoint                    // <u1, c1>
	CarToChargePoint                      // <car1, c1>
	DroneCarToChargePoint                 // <u1, c1, car1>
	WorkerToCharge                        // <w,c>
)
