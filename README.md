## ZST/DME
### refer to the book: Electronic Design Automation: Synthesis, Verification, and Test
### or to the paper by abk: Zero skew clock routing with minimum wirelength (the correct equation for solving X during merge is in the abk paper, the equation in the book and Tsay paper are wrong)
### todo: snaking in top down!!!! and better plotting and blockage avoidance 
### nine region based feasible merge segment cutting: see Through-Silicon-Via-Induced Obstacle-Aware Clock Tree Synthesis for 3D ICs Xin Zhao and Sung Kyu Lim
### todo: snaking in top down!!!! single point test for blockage avoidance
### NNGGRID: see the paper: clock aware low power placement
### todo: calculate time of each part of the algorithm
### added clock tree repair

example command: ./ZSTDME -input ../../Benchmarks/ISPD2009/ispd09f34 -plotPath ../../Graphs/