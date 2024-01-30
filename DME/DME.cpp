#include "DME.h"

void ZSTDMERouter::ZSTDME()
{
    // 1. Build Tree of Segments (bottom up)
    bottomUp();
    draw_bottom_up();

    // 2. Find Exact Placement(top down)
    topDown();

    cout << padding << "Finished DME" << padding << endl;
}

void ZSTDMERouter::topDown()
{
        
    treeNodeLocation.resize(topology->nodeCount);
    solution.resize(topology->nodeCount);
    //auto& rootMergeSegment = vertexMS[topo->root->id];
    Segment& rootMergeSegment=topology->root->trr.core;
    std::function<void(TreeNode*)> preOrderTraversal = [&](TreeNode* curNode) {
        int currentId = curNode->id;

        if (curNode->leftChild != NULL && curNode->rightChild != NULL) {
            // handle curNode
            if (curNode == topology->root) {
                Point_2D tmp;
                // tmp.x = (rootMergeSegment.lowerPoint.x + rootMergeSegment.higherPoint.x) /2;
                // tmp.y = (rootMergeSegment.lowerPoint.y + rootMergeSegment.higherPoint.y) /2;
                db->clockSource = rootMergeSegment.lowerPoint;
                treeNodeLocation[currentId] = rootMergeSegment.lowerPoint;

                //  clockSource = tmp;
                // treeNodeLocation[currentId] = tmp;
            } else {
                TreeNode* parent = curNode->parent;
                int parentId = parent->id;
                //auto& trr_par = vertexTRR[parentId];
                TRR trr_par;
                trr_par.core = Segment(treeNodeLocation[parentId], treeNodeLocation[parentId]);
                
                trr_par.radius = vertexDistE[currentId];
                trr_par.radius=curNode->trr.radius;
                assert(vertexDistE[currentId]==curNode->trr.radius);
                //! vertexDistE[currentId] should equal to curNode->trr.radius here

                // cout <<std::fixed<< "Before merge: the value for trr_par is" << setprecision(2) << trr_par << endl;
                // if(trr_par.radius == 122663.50){
                //     cout << 3 << endl;
                // }
                //Segment merged = trr_par.intersect(vertexMS[currentId]);
                Segment merged = trr_par.intersectTRR(curNode->trr.core);
               
                // if(merged.isLeaf() == false){    
                //     cout << trr_par << " intersecting "<< vertexMS[currentId] <<  endl;
                //     cout << " Not leaf" <<endl;
                //     cout << merged << endl;
                // }
                if (merged.id == -1) {
                    draw_TRR_pair(trr_par,TRR(curNode->trr.core,0));
                    cout << "TRR-MS merging failed" << endl;
                    exit(1);
                }
                treeNodeLocation[currentId] = merged.lowerPoint;//? why lowerPoint? its said that whatever point on ms is ok
            }

            // cout << "Steiner Point " << currentId << " located at " << treeNodeLocation[currentId] << endl;
            preOrderTraversal(curNode->leftChild);
            preOrderTraversal(curNode->rightChild);
        } else {
            // sinks
            //treeNodeLocation[currentId] = vertexMS[currentId].lowerPoint;
            treeNodeLocation[currentId]=curNode->trr.core.lowerPoint;
            return;
        }
    };
    preOrderTraversal(topology->root);
    cout  << "Finish top-down process"  << endl;

    
}

void ZSTDMERouter::bottomUp()
{
        
    std::function<void(TreeNode*)> postOrderTraversal = [&](TreeNode* curNode) {
        int currentId = curNode->id;
        if (curNode->leftChild != NULL && curNode->rightChild != NULL) {//!的确不会有只有一个子节点的中间节点
            //cout<<"dming node: "<<currentId<<endl;
            postOrderTraversal(curNode->leftChild);
            postOrderTraversal(curNode->rightChild);
            // create merging segment for curNode
            //auto& ms_a = vertexMS[curNode->leftChild->id];
            //auto& ms_b = vertexMS[curNode->rightChild->id];

            Segment ms_a = curNode->leftChild->trr.core;
            Segment ms_b = curNode->rightChild->trr.core;
            // get |e_a|, |e_b|
            double d = min(L1Dist(ms_a.lowerPoint, ms_b.lowerPoint), L1Dist(ms_a.lowerPoint, ms_b.higherPoint));
            d = min(d, L1Dist(ms_a.higherPoint, ms_b.lowerPoint));
            d = min(d, L1Dist(ms_a.higherPoint, ms_b.higherPoint));  // but why need to caleftChild 2*2 possiblity?
            
            // double e_a_dist = (ms_b.delay - ms_a.delay + d) / 2;
            // double e_b_dist = (ms_a.delay - ms_b.delay + d) / 2;
            double e_a_dist;
            double e_b_dist;
            //! ea for leftChild and eb for rightChild
            if(delayModel==LINEAR_DELAY)// linear delay model
            {
                e_a_dist = (curNode->rightChild->delay - curNode->leftChild->delay + d) / 2;
                e_b_dist = (curNode->leftChild->delay - curNode->rightChild->delay + d) / 2;
                if (e_a_dist < 0 || e_b_dist < 0) {
                    cout << "Skew too large" << endl;//!
                    exit(1);
                }
            } else if(delayModel==ELMORE_DELAY)// elmore delay(3d)
            {
                double x=calc_x_rightChild(curNode->leftChild,curNode->rightChild,curNode,d);
                if(0 <= x && x <= d){
                    e_a_dist = x;
                    e_b_dist = d - x;
                }
                else if(x < 0){//! 这里是否有假设 a b的相对位置关系？？
                    e_b_dist = calc_L2_rightChild(curNode->leftChild,curNode->rightChild,curNode, 0);
                    //!assert(e_b_dist > d);
                    e_a_dist = 0;
                }
                else if(x > d){
                    e_a_dist = calc_L2_rightChild(curNode->leftChild,curNode->rightChild,curNode, 1);
                    //!assert(e_a_dist > d);
                    e_b_dist = 0;
                }
            }

            Rlc_calculation(curNode,curNode->leftChild,curNode->rightChild,e_a_dist,e_b_dist);

            // todo : this delay should be changed
            // ms_v.delay = e_a_dist + ms_a.delay; 
            // e_a+ms_a is supposed to be equal to e_b+ms_b
            // todo update treenode delay and capacitance, segment should be a member of tree node, and TRR should be a member of segment, what about rebuild the code structure?
            
            update_merge_Capacitance(curNode,curNode->leftChild, curNode->rightChild,e_a_dist,e_b_dist);//? there is a same function in RleftChild_caleftChildulation

            update_merge_Delay(curNode,curNode->leftChild, curNode->rightChild,e_a_dist,e_b_dist);

            vertexDistE[curNode->leftChild->id] = e_a_dist;
            vertexDistE[curNode->rightChild->id] = e_b_dist;

            // get trr_a, trr_b
            //TRR trr_a(ms_a, e_a_dist);
            //TRR trr_b(ms_b, e_b_dist);
            //vertexTRR[curNode->leftChild->id] = trr_a;
            //vertexTRR[curNode->rightChild->id] = trr_b;

            curNode->leftChild->trr.radius=e_a_dist;
            curNode->rightChild->trr.radius=e_b_dist;

            // intersect trr_a, trr_b to get ms_v
            Segment ms_v = TRRintersect(curNode->leftChild->trr, curNode->rightChild->trr);
            // cout << "Merging result: " << ms_v << endl;
            if (ms_v.id == -1) {
                cout<<ms_v<<endl;
                cout<<curNode->leftChild->trr<<endl;
                cout<<curNode->rightChild->trr<<endl;
                cout << "Merge failure" << endl;
                exit(1);
            }

            //! new
            //curNode->load_capacitance=curNode->leftChild->load_capacitance+curNode->rightChild->load_capacitance+c_v*d;//todo: a+b+wire capacitance, and consider snaking?
            //? not d but max(ea,eb)?
            //! new
            vertexMS[currentId] = ms_v;

            curNode->trr.core=ms_v;
            // cout << "Delay diff " << e_a_dist + ms_a.delay - (e_b_dist + ms_b.delay) << endl;
        } else {
            // Create ms for leaf node
            //cout<<"dming leaf: "<<currentId<<endl;
            vertexMS[currentId] = Segment(db->dbSinks[currentId], db->dbSinks[currentId]);
            //vertexMS[currentId] = Segment(taps[currentId]);

            curNode->trr.core = Segment(db->dbSinks[currentId]);
            //cout<<"leaf ttnode id: "<<currentId<<endl;
        }
    };
    postOrderTraversal(topology->root);
    cout  << "Finish bottom-up process"  << endl;
    
}
