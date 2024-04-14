#include "DME.h"

void ZSTDMERouter::ZSTDME()
{
    // 1. Build Tree of Segments (bottom numerator)
    bottomUp();
    drawBottomUp();

    // 2. Find Exact Node Location(top down)
    topDown();

    cout << padding << "Finished DME" << padding << endl;
    buildSolution();
    drawSolution();
}

void ZSTDMERouter::topDown()
{

    treeNodeLocation.resize(topology->nodeCount);
    // auto& rootMergeSegment = vertexMS[topo->root->id];
    Segment &rootMergeSegment = topology->root->trr.core;
    std::function<void(TreeNode *)> preOrderTraversal = [&](TreeNode *curNode)
    {
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        {
            // handle curNode
            if (curNode == topology->root)
            {
                Point_2D tmp;
                // tmp.x = (rootMergeSegment.lowerPoint.x + rootMergeSegment.higherPoint.x) /2;
                // tmp.y = (rootMergeSegment.lowerPoint.y + rootMergeSegment.higherPoint.y) /2;
                db->clockSource = rootMergeSegment.lowerPoint;
                treeNodeLocation[curNode->id] = rootMergeSegment.lowerPoint;

                //  clockSource = tmp;
                // treeNodeLocation[curNode->id] = tmp;
            }
            else
            {
                TRR trr_par;
                trr_par.core = Segment(treeNodeLocation[curNode->parent->id], treeNodeLocation[curNode->parent->id]);
                trr_par.radius = curNode->trr.radius;

                // cout <<std::fixed<< "Before core: the value for trr_par is" << setprecision(2) << trr_par << endl;
                // if(trr_par.radius == 122663.50){
                //     cout << 3 << endl;
                // }
                // Segment merged = trr_par.intersect(vertexMS[curNode->id]);
                Segment merged = TRRintersectSeg(trr_par, curNode->trr.core);

                // if(merged.isLeaf() == false){
                //     cout << trr_par << " intersecting "<< vertexMS[curNode->id] <<  endl;
                //     cout << " Not leaf" <<endl;
                //     cout << merged << endl;
                // }
                if (merged.id == -1)
                {
                    drawTRRPair("bottomup", curNode->trr, curNode->parent->trr);
                    drawTRRPair("topdown", trr_par, TRR(curNode->trr.core, 0.0));
                    cout << "child trr core: " << curNode->trr.core << endl;
                    cout << "parent trr : " << trr_par << endl;
                    cout << "TRR-MS insersection not found" << endl;
                    exit(1);
                }
                treeNodeLocation[curNode->id] =
                    merged.lowerPoint; //! why lowerPoint? its said that
                                       //! whatever point on ms is ok, but
                                       //! different choices would determine if
                                       //! we need snaking or not, see the book
                                       //! for details

                if (double_less(L1Dist(treeNodeLocation[curNode->parent->id], treeNodeLocation[curNode->id]), curNode->trr.radius))
                {
                    double snakingWirelength = curNode->trr.radius - L1Dist(treeNodeLocation[curNode->parent->id], treeNodeLocation[curNode->id]);
                    //? how to do exact snaking espacially when there are blockages?
                    //? how to add snaking here????
                    // todo: add snaking
                }
            }

            // cout << "Steiner Point " << curNode->id << " located at " << treeNodeLocation[curNode->id] << endl;
            preOrderTraversal(curNode->leftChild);
            preOrderTraversal(curNode->rightChild);
        }
        else
        {
            treeNodeLocation[curNode->id] = curNode->trr.core.lowerPoint;
            return;
        }
    };
    preOrderTraversal(topology->root);
    cout << "Finish top-down process" << endl;
}

void ZSTDMERouter::bottomUp()
{
    int index = 0;
    std::function<void(TreeNode *)> postOrderTraversal = [&](TreeNode *curNode)
    {
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        { //! 的确不会有只有一个子节点的中间节点

            postOrderTraversal(curNode->leftChild);
            postOrderTraversal(curNode->rightChild);

            index++;
            // create merging segment for curNode
            // Segment ms_a = curNode->leftChild->trr.core;
            // Segment ms_b = curNode->rightChild->trr.core;

            //! L denote the shortest Manhattan distance between ms_a and ms_b, that is, d(ms_a,ms_b),
            //! which is also denoted as κ in the abk paper
            double L = minManhattanDist(curNode->leftChild, curNode->rightChild);
            double e_a; // radius of TRR_a
            double e_b; // radius of TRR_b, e_a, e_b are used in the book or the abk paper as well
            //! e_a for leftChild and e_b for rightChild
            if (delayModel == LINEAR_DELAY) // linear delay model, see the book or the abk paper
            {
                e_a = (curNode->rightChild->delay - curNode->leftChild->delay + L) / 2;
                e_b = (curNode->leftChild->delay - curNode->rightChild->delay + L) / 2;
                if (double_less(e_a, 0.0) || double_less(e_b, 0.0))
                {
                    if (double_less(e_a, 0.0))
                    {
                        assert(double_greater(e_b, 0.0));
                        e_a = 0.0;
                        e_b = curNode->leftChild->delay - curNode->rightChild->delay;
                        // so e_a + ta = e_b + tb
                    }
                    else
                    {
                        assert(double_greater(e_a, 0.0));
                        e_b = 0.0;
                        e_b = curNode->rightChild->delay - curNode->leftChild->delay;
                    }
                }
            }
            else if (delayModel == ELMORE_DELAY) // elmore delay, see the book for clear explanation
            {
                double x = solveForX_multiMetal(curNode->leftChild, curNode->rightChild, curNode, L, db->dbMetals, db->TSV);
                if (double_greaterorequal(x, 0.0) && double_lessorequal(x, L))
                {
                    e_a = x;
                    e_b = L - x;
                }
                else if (double_less(x, 0.0))
                {
                    e_a = 0.0;
                    e_b = solveForLPrime_multiMetal(curNode->leftChild, curNode->rightChild, curNode, 0, db->dbMetals, db->TSV);
                    assert(double_greater(e_b, L));
                }
                else if (double_less(L, x))
                {
                    e_b = 0.0;
                    e_a = solveForLPrime_multiMetal(curNode->leftChild, curNode->rightChild, curNode, 1, db->dbMetals, db->TSV);
                    assert(double_greater(e_a, L));
                }
            }
            else
            {
                cout << RED << "SET DELAY MODEL FIRST!" << RESET << endl;
                exit(0);
            }

            //! recalculate ea and eb considering L based on the results using elmore delay
            RLCCalculation(curNode, curNode->leftChild, curNode->rightChild, e_a, e_b, db->dbMetals, db->TSV);

            curNode->leftChild->trr.radius = e_a; //! e_a for leftChild and e_b for rightChild
            curNode->rightChild->trr.radius = e_b;

            // intersect trr_a, trr_b to get ms_v
            Segment ms_v = TRRintersectTRR(curNode->leftChild->trr, curNode->rightChild->trr);

            // todo: nine region method to solve for feasible merging segment, and recalculate ea and eb

            if (ms_v.id == -1)
            {
                cout << "Merge failure" << endl;
                cout << "TRR 1: " << curNode->leftChild->trr << endl;
                cout << "TRR 2: " << curNode->rightChild->trr << endl;

                exit(1);
            }
            curNode->trr.core = ms_v;

            // updateMergeDelay_multiMetal(curNode, curNode->leftChild, curNode->rightChild, e_a, e_b, db->dbMetals, db->TSV);
            updateMergeCapacitance_multiMetal(curNode, curNode->leftChild, curNode->rightChild, e_a, e_b, db->dbMetals, db->TSV);
        }
        else
        {
            // Create ms for leaf node
            assert(curNode);
            assert(curNode->id < db->dbSinks.size());
            curNode->trr.core = Segment(db->dbSinks[curNode->id]);
        }
    };
    postOrderTraversal(topology->root);
    cout << "Finish bottom-up process" << endl;
}

void ZSTDMERouter::repairSolution()
{
    bool obstacleAware = false;
    if (db->dbBlockages.size() > 0)
    {
        obstacleAware = true;
    }
    if (gArg.CheckExist("ignoreBlockage"))
    {
        obstacleAware = false;
    }
    // assumption: source is not inside any blockage, because its a merge point and is already dealt in buildSolution
    //! important: after moving merge points out of blockages, any steiner point inside blockage mustn't be a merge point and must be a bend point(for L shape)!
    if (obstacleAware)
    {
        std::function<void(SteinerPoint *)> traceToSource = [&](SteinerPoint *curNode)
        {
            if (curNode->parent == NULL)
            { // reached source
                return;
            }
            // Step 1: repair full crossings
            RectilinearLine curLine(*curNode, *curNode->parent);
            // sort blockages by x/y, and consider direction of curLine: going left/right, up/down
            for (Blockage curBlockage : db->dbBlockages)
            {
                if (curLine.direction == HORIZONTAL)
                {
                    Interval height(curBlockage.ll.y, curBlockage.ur.y);

                    if (height.inside(curLine.ll.y))
                    {
                        Interval width(curBlockage.ll.x, curBlockage.ur.x);
                        Interval curLineLength(curLine.ll.x, curLine.ur.x);
                        if (curLineLength.include(width))
                        {
                            // detour, with the nearest horizontal boundary of the blockage(top or bottom boundary)
                        }
                    }
                }
                else if (curLine.direction == VERTICAL)
                {
                }
                else
                {
                    cerr << "RECTLINEAR LINE DIRECTION ERROR\n";
                    exit(0);
                }
            }

            // Step 2: repair bends
            auto &nxtNode = curNode->parent;
            traceToSource(nxtNode);
        };

        // todo potential Step 3: repair skew by snaking!
    }
}

void ZSTDMERouter::drawBottomUp()
{
    string plotPath;
    string benchmarkName;
    if (!gArg.GetString("plotPath", &plotPath))
    {
        plotPath = "./";
    }
    gArg.GetString("benchmarkName", &benchmarkName);

    string outFilePath = plotPath + benchmarkName + "_bottom_up.plt";
    ofstream outfile(outFilePath.c_str(), ios::out);

    outfile << " " << endl;
    outfile << "set terminal png size 4000,4000" << endl;
    outfile << "set output "
            << "\"" << plotPath << benchmarkName + "_bottom_up"
            << ".png\"" << endl;
    // outfile << "set multiplot layout 1, 2" << endl;
    outfile << "set size ratio -1" << endl;
    outfile << "set nokey" << endl
            << endl;

    // for(int i=0; i<cell_list_top.size(); i++){
    //     outfile << "set label " << i + 2 << " \"" << cell_list_top[i]->get_name() << "\" at " << cell_list_top[i]->get_posX() + cell_list_top[i]->get_width() / 2 << "," << cell_list_top[i]->get_posY() + cell_list_top[i]->get_height() / 2 << " center front" << endl;
    // }
    // outfile << "set xrange [0:" << _pChip->get_width() << "]" << endl;
    // outfile << "set yrange [0:" << _pChip->get_height() << "]" << endl;
    // outfile << "plot[:][:] '-' w l lt 3 lw 2, '-' with filledcurves closed fc \"grey90\" fs border leftChild \"red\", '-' with filledcurves closed fc \"yellow\" fs border leftChild \"black\", '-' w l lt 1" << endl << endl;

    outfile << "plot[:][:]  '-' w l lt 3 lw 2, '-'  w l lt 7 lw 10, '-' w p pt 7 ps 2" << endl
            << endl;

    outfile << "# TRR" << endl;
    std::function<void(TreeNode *)> postOrderTraversal = [&](TreeNode *curNode)
    {
        int curId = curNode->id;
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        {
            postOrderTraversal(curNode->leftChild);
            postOrderTraversal(curNode->rightChild);
            curNode->trr.drawTRR(outfile);
            return;
        }
        else
        {
            curNode->trr.drawTRR(outfile);
        }
    };
    postOrderTraversal(topology->root);
    outfile << "EOF" << endl;

    outfile << "# TRR cores" << endl;
    std::function<void(TreeNode *)> postOrderTraversal_core = [&](TreeNode *curNode)
    {
        int curId = curNode->id;
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        {
            postOrderTraversal_core(curNode->leftChild);
            postOrderTraversal_core(curNode->rightChild);
            curNode->trr.drawCore(outfile);
            return;
        }
        else
        {
            // curNode->trr.draw_core(outfile);
        }
    };
    postOrderTraversal_core(topology->root);
    outfile << "EOF" << endl;

    outfile << "# Sinks" << endl;
    std::function<void(TreeNode *)> postOrderTraversal_sink = [&](TreeNode *curNode)
    {
        int curId = curNode->id;
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        {
            postOrderTraversal_sink(curNode->leftChild);
            postOrderTraversal_sink(curNode->rightChild);
            return;
        }
        else
        {
            curNode->trr.drawCore(outfile);
        }
    };
    postOrderTraversal_sink(topology->root);
    outfile << "EOF" << endl;

    // outfile << "pause -1 'Press any key to close.'" << endl;
    outfile.close();

    system(("gnuplot " + outFilePath).c_str());

    cout << BLUE << "[Router]" << RESET << " - Visualize the bottom_up graph in \'" << outFilePath << "\'.\n";
}

void ZSTDMERouter::drawBottomUpMerge(string name, TRR trr1, TRR trr2, Segment merge)
{
    string plotPath;
    string benchmarkName;
    if (!gArg.GetString("plotPath", &plotPath))
    {
        plotPath = "./";
    }
    gArg.GetString("benchmarkName", &benchmarkName);

    string outFilePath = plotPath + benchmarkName + "_bottom_up_" + name + ".plt";
    ofstream outfile(outFilePath.c_str(), ios::out);

    outfile << " " << endl;
    outfile << "set terminal png size 4000,4000" << endl;
    outfile << "set output "
            << "\"" << plotPath << benchmarkName + "_bottom_up" + name
            << ".png\"" << endl;
    // outfile << "set multiplot layout 1, 2" << endl;
    outfile << "set size ratio -1" << endl;
    outfile << "set nokey" << endl
            << endl;

    // for(int i=0; i<cell_list_top.size(); i++){
    //     outfile << "set label " << i + 2 << " \"" << cell_list_top[i]->get_name() << "\" at " << cell_list_top[i]->get_posX() + cell_list_top[i]->get_width() / 2 << "," << cell_list_top[i]->get_posY() + cell_list_top[i]->get_height() / 2 << " center front" << endl;
    // }
    // outfile << "set xrange [0:" << _pChip->get_width() << "]" << endl;
    // outfile << "set yrange [0:" << _pChip->get_height() << "]" << endl;
    // outfile << "plot[:][:] '-' w l lt 3 lw 2, '-' with filledcurves closed fc \"grey90\" fs border leftChild \"red\", '-' with filledcurves closed fc \"yellow\" fs border leftChild \"black\", '-' w l lt 1" << endl << endl;

    outfile << "plot[:][:]  '-' w l lt 3 lw 2, '-'  w l lt 7 lw 16, '-'  w l lt 7 lw 16 " << endl
            << endl;

    outfile << "# TRR" << endl;
    trr1.drawTRR(outfile);
    trr2.drawTRR(outfile);
    outfile << "EOF" << endl;

    outfile << "# TRR cores" << endl;
    trr1.drawCore(outfile);
    trr2.drawCore(outfile);
    outfile << "EOF" << endl;

    outfile << "# Merge segments" << endl;
    plotLinePLT(outfile, merge.lowerPoint.x,
                merge.lowerPoint.y,
                merge.higherPoint.x,
                merge.higherPoint.y);
    outfile << "EOF" << endl;

    // outfile << "pause -1 'Press any key to close.'" << endl;
    outfile.close();

    system(("gnuplot " + outFilePath).c_str());

    cout << BLUE << "[Router]" << RESET << " - Visualize the bottom_up graph in \'" << outFilePath << "\'.\n";
}

void ZSTDMERouter::buildSolution()
{
    bool fullSolution = true; //! include L-shapes or include L-shapes as “diagonal wires” to reduce clutter
    bool obstacleAware = false;
    if (gArg.CheckExist("diagonalSolution"))
    {
        fullSolution = false;
    }
    if (db->dbBlockages.size() > 0)
    {
        obstacleAware = true;
    }
    if (gArg.CheckExist("ignoreBlockage"))
    {
        obstacleAware = false;
    }

    // preorder traversal to buil grsteiner structure
    solution.resize(topology->nodeCount);
    std::function<void(TreeNode *)> preOrderTraversal = [&](TreeNode *curNode)
    {
        int curId = curNode->id;
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        {
            // handle curNode
            SteinerPoint *curSteiner = solution[curId];
            auto &lc = curNode->leftChild;
            auto &rc = curNode->rightChild;

            Point_2D leftChildLocation = treeNodeLocation[curNode->leftChild->id];
            Point_2D rightChildLocation = treeNodeLocation[curNode->rightChild->id];

            //! move merge point out of obstacles if there are obstacles, then the tree will be repaired in repairSolution
            if (obstacleAware)
            {

                for (Blockage curBlockage : db->dbBlockages)
                {
                    if (!lc->isLeaf())
                    {

                        //! move the merge point to the nearest boundary of the obstacle
                        leftChildLocation = moveToClosestBoundary(leftChildLocation, curBlockage); // todo early exit here, because a point can only be inside of one blockage
                    }
                    if (!rc->isLeaf())
                    {

                        rightChildLocation = moveToClosestBoundary(rightChildLocation, curBlockage);
                    }
                }
            }
            SteinerPoint *lcSteiner = new SteinerPoint(leftChildLocation);
            SteinerPoint *rcSteiner = new SteinerPoint(rightChildLocation);

            curSteiner->actualMergeNode = true;
            lcSteiner->actualMergeNode = true;
            rcSteiner->actualMergeNode = true;

            setSteinerNode(lcSteiner, lc);
            setSteinerNode(rcSteiner, rc);
            setSteinerNode(curSteiner, curNode);

            if (fullSolution)
            {
                // Connect lc
                assert(double_greaterorequal(lc->trr.radius, minManhattanDist(*lcSteiner, *curSteiner)));
                assert(double_greaterorequal(rc->trr.radius, minManhattanDist(*rcSteiner, *curSteiner)));
                if (!(double_greater(lc->trr.radius, minManhattanDist(*lcSteiner, *curSteiner)))) // no snaking for lc
                // if(1)
                {
                    if (double_equal(curSteiner->x, lcSteiner->x) || double_equal(curSteiner->y, lcSteiner->y))
                    {
                        lcSteiner->setParent(curSteiner);
                    }
                    else
                    { // Use L-shape
                        SteinerPoint *middle = new SteinerPoint(Point_2D(curSteiner->x, lcSteiner->y));
                        lcSteiner->setParent(middle);
                        middle->setParent(curSteiner);
                        middle->metalLayerIndex = curSteiner->metalLayerIndex;
                    }
                }
                else
                {
                    //! snaking
                    double snakingWirelength = lc->trr.radius - minManhattanDist(*lcSteiner, *curSteiner);
                    cout << "snaking!\n";
                    SteinerPoint *snakePointForCur;
                    SteinerPoint *snakePointForLeftChild;
                    if (double_equal(curSteiner->x, lcSteiner->x))
                    {
                        // boundary check? make sure wire doesn't go outside the chip area
                        snakePointForCur = new SteinerPoint(Point_2D(curSteiner->x + 0.5 * snakingWirelength, curSteiner->y));
                        snakePointForLeftChild = new SteinerPoint(Point_2D(lcSteiner->x + 0.5 * snakingWirelength, lcSteiner->y));
                        lcSteiner->setParent(snakePointForLeftChild);
                        snakePointForLeftChild->setParent(snakePointForCur);
                        snakePointForCur->setParent(curSteiner);
                    }
                    else if (double_equal(curSteiner->y, lcSteiner->y))
                    {
                        snakePointForCur = new SteinerPoint(Point_2D(curSteiner->x, curSteiner->y + 0.5 * snakingWirelength));
                        snakePointForLeftChild = new SteinerPoint(Point_2D(lcSteiner->x, lcSteiner->y + 0.5 * snakingWirelength));
                        lcSteiner->setParent(snakePointForLeftChild);
                        snakePointForLeftChild->setParent(snakePointForCur);
                        snakePointForCur->setParent(curSteiner);
                    }
                    else
                    {
                        // Use L-shape
                        if (double_greater(curSteiner->x, lcSteiner->x)) // curSteiner at right of lc
                        {
                            snakePointForCur = new SteinerPoint(Point_2D(curSteiner->x + 0.5 * snakingWirelength, curSteiner->y));
                            snakePointForLeftChild = new SteinerPoint(Point_2D(curSteiner->x + 0.5 * snakingWirelength, lcSteiner->y));
                            lcSteiner->setParent(snakePointForLeftChild);
                            snakePointForLeftChild->setParent(snakePointForCur);
                            snakePointForCur->setParent(curSteiner);
                        }
                        else
                        {
                            snakePointForCur = new SteinerPoint(Point_2D(curSteiner->x - 0.5 * snakingWirelength, curSteiner->y));
                            snakePointForLeftChild = new SteinerPoint(Point_2D(curSteiner->x - 0.5 * snakingWirelength, lcSteiner->y));
                            lcSteiner->setParent(snakePointForLeftChild);
                            snakePointForLeftChild->setParent(snakePointForCur);
                            snakePointForCur->setParent(curSteiner);
                        }
                    }
                    snakePointForCur->snakeNode = true;
                    snakePointForLeftChild->snakeNode = true;
                    snakePointForCur->metalLayerIndex = curSteiner->metalLayerIndex;
                    snakePointForLeftChild->metalLayerIndex = curSteiner->metalLayerIndex;
                }

                // Connect rc
                if (!(double_greater(rc->trr.radius, minManhattanDist(*rcSteiner, *curSteiner)))) // no snaking for rc
                // if(1)
                {
                    if (double_equal(curSteiner->x, rcSteiner->x) || double_equal(curSteiner->y, rcSteiner->y))
                    {
                        rcSteiner->setParent(curSteiner);
                    }
                    else
                    { // Use L-shape
                        SteinerPoint *middle = new SteinerPoint(Point_2D(curSteiner->x, rcSteiner->y));
                        rcSteiner->setParent(middle);
                        middle->setParent(curSteiner);
                        middle->metalLayerIndex = curSteiner->metalLayerIndex;
                    }
                }
                else
                {
                    //! snaking
                    double snakingWirelength = rc->trr.radius - minManhattanDist(*rcSteiner, *curSteiner);
                    cout << "snaking!\n";
                    SteinerPoint *snakePointForCur;
                    SteinerPoint *snakePointForRightChild;
                    if (double_equal(curSteiner->x, rcSteiner->x))
                    {
                        // boundary check? make sure wire doesn't go outside the chip area
                        snakePointForCur = new SteinerPoint(Point_2D(curSteiner->x + 0.5 * snakingWirelength, curSteiner->y));
                        snakePointForRightChild = new SteinerPoint(Point_2D(rcSteiner->x + 0.5 * snakingWirelength, rcSteiner->y));
                        rcSteiner->setParent(snakePointForRightChild);
                        snakePointForRightChild->setParent(snakePointForCur);
                        snakePointForCur->setParent(curSteiner);
                    }
                    else if (double_equal(curSteiner->y, rcSteiner->y))
                    {
                        snakePointForCur = new SteinerPoint(Point_2D(curSteiner->x, curSteiner->y + 0.5 * snakingWirelength));
                        snakePointForRightChild = new SteinerPoint(Point_2D(rcSteiner->x, rcSteiner->y + 0.5 * snakingWirelength));
                        rcSteiner->setParent(snakePointForRightChild);
                        snakePointForRightChild->setParent(snakePointForCur);
                        snakePointForCur->setParent(curSteiner);
                    }
                    else
                    {
                        // Use L-shape
                        if (double_greater(curSteiner->x, rcSteiner->x)) // curSteiner at right of rc
                        {
                            snakePointForCur = new SteinerPoint(Point_2D(curSteiner->x + 0.5 * snakingWirelength, curSteiner->y));
                            snakePointForRightChild = new SteinerPoint(Point_2D(curSteiner->x + 0.5 * snakingWirelength, rcSteiner->y));
                            rcSteiner->setParent(snakePointForRightChild);
                            snakePointForRightChild->setParent(snakePointForCur);
                            snakePointForCur->setParent(curSteiner);
                        }
                        else
                        {
                            snakePointForCur = new SteinerPoint(Point_2D(curSteiner->x - 0.5 * snakingWirelength, curSteiner->y));
                            snakePointForRightChild = new SteinerPoint(Point_2D(curSteiner->x - 0.5 * snakingWirelength, rcSteiner->y));
                            rcSteiner->setParent(snakePointForRightChild);
                            snakePointForRightChild->setParent(snakePointForCur);
                            snakePointForCur->setParent(curSteiner);
                        }
                    }
                    snakePointForCur->snakeNode = true;
                    snakePointForRightChild->snakeNode = true;
                    snakePointForCur->metalLayerIndex = curSteiner->metalLayerIndex;
                    snakePointForRightChild->metalLayerIndex = curSteiner->metalLayerIndex;
                }
            }
            else
            {
                // Connect lc
                lcSteiner->setParent(curSteiner);

                // Connect rc
                rcSteiner->setParent(curSteiner);
            }
            solution[curNode->leftChild->id] = lcSteiner;
            solution[curNode->rightChild->id] = rcSteiner;
            preOrderTraversal(curNode->leftChild);
            preOrderTraversal(curNode->rightChild);
        }
        else
        {
            // sinks
            // pl[curId] = vertexMS[curId].p1;
            return;
        }
    };

    if (obstacleAware)
    {
        for (Blockage curBlockage : db->dbBlockages)
        {

            treeNodeLocation[topology->root->id] = moveToClosestBoundary(treeNodeLocation[topology->root->id], curBlockage);
        }
    }

    solution[topology->root->id] = new SteinerPoint(treeNodeLocation[topology->root->id]);
    preOrderTraversal(topology->root);
}

void ZSTDMERouter::drawSolution()
{

    string plotPath;
    string benchmarkName;
    if (!gArg.GetString("plotPath", &plotPath))
    {
        plotPath = "./";
    }
    gArg.GetString("benchmarkName", &benchmarkName);

    string outFilePath = plotPath + benchmarkName + "_solution.plt";
    ofstream outfile(outFilePath.c_str(), ios::out);

    outfile << " " << endl;
    outfile << "set terminal png size 4000,4000" << endl;
    outfile << "set output "
            << "\"" << plotPath << benchmarkName + "_solution"
            << ".png\"" << endl;
    // outfile << "set multiplot layout 1, 2" << endl;
    outfile << "set size ratio -1" << endl;
    outfile << "set nokey" << endl
            << endl;

    // for(int i=0; i<cell_list_top.size(); i++){
    //     outfile << "set label " << i + 2 << " \"" << cell_list_top[i]->get_name() << "\" at " << cell_list_top[i]->get_posX() + cell_list_top[i]->get_width() / 2 << "," << cell_list_top[i]->get_posY() + cell_list_top[i]->get_height() / 2 << " center front" << endl;
    // }
    // outfile << "set xrange [0:" << _pChip->get_width() << "]" << endl;
    // outfile << "set yrange [0:" << _pChip->get_height() << "]" << endl;
    // outfile << "plot[:][:] '-' w l lt 3 lw 2, '-' with filledcurves closed fc \"grey90\" fs border lc \"red\", '-' with filledcurves closed fc \"yellow\" fs border lc \"black\", '-' w l lt 1" << endl << endl;

    outfile << "plot[:][:]  '-' w l lt 3 lw 2, '-' w l lt 4 lw 2, '-' w p pt 6 ps 2, '-' w p pt 7 ps 1, '-' w p pt 5 ps 2, '-' w l lt 4 lw 2, " << endl
            << endl;

    outfile << "# TREE NONE SNAKE WIRES" << endl;
    std::function<void(SteinerPoint *)> traceToSource = [&](SteinerPoint *curNode)
    {
        if (curNode->parent == NULL)
        { // reached source
            return;
        }
        auto &nxtNode = curNode->parent;
        if (!(curNode->snakeNode ^ nxtNode->snakeNode))
            plotLinePLT(outfile, curNode->x, curNode->y, nxtNode->x, nxtNode->y);
        traceToSource(nxtNode);
    };
    for (int tapId = 0; tapId < db->dbSinks.size(); tapId++)
    {
        traceToSource(solution[tapId]);
    }
    outfile << "EOF" << endl;

    outfile << "# TREE SNAKING WIRES" << endl;
    std::function<void(SteinerPoint *)> traceToSource_SNAKE = [&](SteinerPoint *curNode)
    {
        if (curNode->parent == NULL)
        { // reached source
            return;
        }
        auto &nxtNode = curNode->parent;
        if ((curNode->snakeNode ^ nxtNode->snakeNode))
            plotLinePLT(outfile, curNode->x, curNode->y, nxtNode->x, nxtNode->y);
        traceToSource_SNAKE(nxtNode);
    };
    for (int tapId = 0; tapId < db->dbSinks.size(); tapId++)
    {
        traceToSource_SNAKE(solution[tapId]);
    }
    outfile << "EOF" << endl;

    outfile << "# MergeNodes" << endl;
    std::function<void(TreeNode *)> postOrderTraversal_merge = [&](TreeNode *curNode)
    {
        int curId = curNode->id;
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        {
            postOrderTraversal_merge(curNode->leftChild);
            postOrderTraversal_merge(curNode->rightChild);
            plotLinePLT(outfile, treeNodeLocation[curNode->id].x, treeNodeLocation[curNode->id].y, treeNodeLocation[curNode->id].x, treeNodeLocation[curNode->id].y);
        }
        else
        {
            return;
        }
    };
    postOrderTraversal_merge(topology->root);
    outfile << "EOF" << endl;

    outfile << "# Sinks" << endl;
    std::function<void(TreeNode *)> postOrderTraversal_sink = [&](TreeNode *curNode)
    {
        int curId = curNode->id;
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        {
            postOrderTraversal_sink(curNode->leftChild);
            postOrderTraversal_sink(curNode->rightChild);
            return;
        }
        else
        {
            curNode->trr.drawCore(outfile);
        }
    };
    postOrderTraversal_sink(topology->root);
    outfile << "EOF" << endl;

    outfile << "# Source" << endl;
    plotLinePLT(outfile, treeNodeLocation[topology->root->id].x, treeNodeLocation[topology->root->id].y, treeNodeLocation[topology->root->id].x, treeNodeLocation[topology->root->id].y);
    outfile << "EOF" << endl;

    outfile << "# Blockages" << endl;
    for (Blockage block : this->db->dbBlockages)
    {
        plotBoxPLT(outfile, block.ll.x, block.ll.y, block.ur.x, block.ll.y, block.ur.x, block.ur.y, block.ll.x, block.ur.y); // counter clock-wise
    }
    outfile << "EOF" << endl;

    // outfile << "pause -1 'Press any key to close.'" << endl;
    outfile.close();

    system(("gnuplot " + outFilePath).c_str());

    cout << BLUE << "[Router]" << RESET << " - Visualize the solution graph in \'" << outFilePath << "\'.\n";
}

void ZSTDMERouter::buildSolution_ISPD()
{
    // todo: implement me
    vector<wire> wires;
    vector<wire> bufferWires;

    vector<pair<int, Point_2D>> bufferNodes; // not inserted into the tree, just for output

    set<int> addedTSV;    // store nodenames(//!must be actual merge node!) if a nodename is in the set, there is a tsv between it and its parent
    set<int> addedBuffer; // similar with addedTSV
    set<int> addedWire;   // wires from this node to its parent is already added

    wires.clear();
    bufferWires.clear();

    // here the ID is used as nodename in the output file, and should start from 1 because nodename 0 is reserved for source
    map<SteinerPoint *, int> steinerPointToIdMap;
    map<int, SteinerPoint *> IDtoSteinerPointMap;

    int topoNodeCount = topology->nodeCount;
    assert(topology->nodeCount == solution.size());

    ///////////////////////////////////////////////////////
    // !Step 0: assign node name for sink and merge nodes
    ///////////////////////////////////////////////////////

    for (int i = 0; i < topoNodeCount; i++)
    {
        SteinerPoint *curSteiner = solution[i];
        steinerPointToIdMap[curSteiner] = i + 1; // nodename 0 is reserved for source and source inverter
        IDtoSteinerPointMap[i + 1] = curSteiner;

        // so for sink nodes, ID(nodeName) = treeNode.id + 1, since i == treeNode.id, here treeNode is the corresponding tree node of the steiner node in the vector solution
    }

    ///////////////////////////////////////////////////////
    // !Step 1: assign node name for non merge/sink nodes
    ///////////////////////////////////////////////////////

    int nodeName = topoNodeCount + 1; // node name should start form topoNodeCount+1 now; //! this variable is used for all following operations!!!

    std::function<void(SteinerPoint *)> traceToSource_assignNodeName = [&](SteinerPoint *curNode)
    {
        if (curNode->parent == NULL)
        { // reached source
            return;
        }

        if (steinerPointToIdMap.find(curNode) == steinerPointToIdMap.end())
        {
            // curNode is a snaking or L shape node that was not assigned a node name
            steinerPointToIdMap[curNode] = nodeName;
            IDtoSteinerPointMap[nodeName] = curNode;
            nodeName++;
        }

        auto &nxtNode = curNode->parent;
        traceToSource_assignNodeName(nxtNode);
    };
    for (int tapId = 0; tapId < db->dbSinks.size(); tapId++)
    {
        traceToSource_assignNodeName(solution[tapId]);
    }

    ///////////////////////////////////////////////////////
    // !Step 2: add TSV nodes and assign node name for TSV nodes
    ///////////////////////////////////////////////////////
    //? use pseudo nodes, just like buffer nodes?
    std::function<void(SteinerPoint *)> traceToSource_addTSVNode = [&](SteinerPoint *curNode)
    {
        //! TSV node is added right before next merge node!
        if (curNode->parent == NULL)
        { // reached source
            return;
        }
        //! inserting TSV
        assert(curNode->actualMergeNode && !curNode->snakeNode);
        assert(curNode->parent);
        SteinerPoint *nextMergeNode;
        SteinerPoint *TSV = NULL;

        if (curNode->parent->snakeNode) // snaking, and there must be 2 snaking nodes
        {
            assert(curNode->parent->parent->snakeNode);
            // cur -> snakenode1 -> snakenode2 -> mergenode
            SteinerPoint *snakeNodeBeforeNextMergeNode = curNode->parent->parent;
            nextMergeNode = snakeNodeBeforeNextMergeNode->parent;
            if (nextMergeNode->isTSVNode) // TSV already inserted
            {
                // nextMergeNode=TSV node if TSV already inserted
                nextMergeNode = nextMergeNode->parent;
                assert(nextMergeNode->actualMergeNode);
            }
            else if (!(curNode->layer == nextMergeNode->layer)) // need TSV
            {
                if (addedTSV.find(steinerPointToIdMap[curNode]) == addedTSV.end()) // TSV not inserted yet
                {
                    Point_2D TSVnodeLocation = determineTSVLocation(snakeNodeBeforeNextMergeNode, nextMergeNode);

                    TSV = new SteinerPoint(TSVnodeLocation);

                    snakeNodeBeforeNextMergeNode->parent = TSV;

                    TSV->isTSVNode = true;
                    TSV->metalLayerIndex = db->dbMetals.size(); //!!!
                    TSV->parent = nextMergeNode;
                    addedTSV.insert(steinerPointToIdMap[curNode]); //! curNode as first, nextMergeNode as second!
                }
            }
            assert(nextMergeNode->actualMergeNode);
        }
        else if (curNode->parent->actualMergeNode)
        {
            // cur -> mergenode
            nextMergeNode = curNode->parent;
            if (nextMergeNode->isTSVNode) // TSV already inserted
            {
                // nextMergeNode=TSV node if TSV already inserted
                nextMergeNode = nextMergeNode->parent;
                assert(nextMergeNode->actualMergeNode);
            }
            else if (!(curNode->layer == nextMergeNode->layer))
            {
                if (addedTSV.find(steinerPointToIdMap[curNode]) == addedTSV.end()) // TSV not inserted yet
                {
                    assert(nextMergeNode->actualMergeNode);
                    Point_2D TSVnodeLocation = determineTSVLocation(curNode, nextMergeNode);

                    TSV = new SteinerPoint(Point_2D(TSVnodeLocation));

                    curNode->parent = TSV;

                    TSV->isTSVNode = true;
                    TSV->metalLayerIndex = db->dbMetals.size(); //!!!
                    TSV->parent = nextMergeNode;
                    addedTSV.insert(steinerPointToIdMap[curNode]); //! curNode as first, nextMergeNode as second!
                }
            }
            assert(nextMergeNode->actualMergeNode);
        }
        else // no snaking, just L shape
        {
            // cur -> middleNodeForLshape -> mergenode
            assert(!curNode->parent->actualMergeNode);
            SteinerPoint *middleNode = curNode->parent;
            nextMergeNode = middleNode->parent;
            cout << *curNode << " " << *curNode->parent << " " << *curNode->parent->parent << endl;
            cout << curNode->actualMergeNode << " " << middleNode->actualMergeNode << " " << nextMergeNode->actualMergeNode << endl;
            cout << curNode->actualMergeNode << " " << middleNode->layer << " " << nextMergeNode->layer << endl;
            if (nextMergeNode->isTSVNode) // TSV already inserted
            {
                // nextMergeNode=TSV node if TSV already inserted
                nextMergeNode = nextMergeNode->parent;
                assert(nextMergeNode->actualMergeNode);
            }
            else if (!(curNode->layer == nextMergeNode->layer))
            {
                // if (addedTSV.find(make_pair(steinerPointToIdMap[curNode], steinerPointToIdMap[nextMergeNode])) == addedTSV.end()) // TSV not inserted yet
                if (addedTSV.find(steinerPointToIdMap[curNode]) == addedTSV.end()) // TSV not inserted yet
                {
                    assert(nextMergeNode->actualMergeNode);
                    Point_2D TSVnodeLocation = determineTSVLocation(middleNode, nextMergeNode);

                    TSV = new SteinerPoint(Point_2D(TSVnodeLocation));

                    middleNode->parent = TSV;

                    TSV->isTSVNode = true;
                    TSV->metalLayerIndex = db->dbMetals.size(); //!!!
                    TSV->parent = nextMergeNode;

                    // addedTSV.insert(make_pair(steinerPointToIdMap[curNode], steinerPointToIdMap[nextMergeNode])); //! curNode as first, nextMergeNode as second!
                    addedTSV.insert(steinerPointToIdMap[curNode]);
                    //// 保持curNode的node name在前，这样能够避免重复吗？即 （1,2）,(2,1)类的重复
                }
            }
            assert(nextMergeNode->actualMergeNode);
        }
        //! TSV inserted
        if (TSV)
        {
            steinerPointToIdMap[TSV] = nodeName;
            IDtoSteinerPointMap[nodeName] = TSV;
            nodeName++;
        }

        traceToSource_addTSVNode(nextMergeNode);
    };
    for (int tapId = 0; tapId < db->dbSinks.size(); tapId++)
    {
        traceToSource_addTSVNode(solution[tapId]);
    }
    int nonBufferNodeCount = nodeName - 1;
    //! be aware of wire direction
    ///////////////////////////////////////////////////////
    // !Step 3: add buffer nodes(pseudo, not inserted to the steiner tree) and buffer wires
    ///////////////////////////////////////////////////////
    std::function<void(SteinerPoint *)> traceToSource_addBuffers = [&](SteinerPoint *curNode)
    {
        if (curNode->parent == NULL)
        { // reached source
            return;
        }

        assert(curNode->actualMergeNode);

        SteinerPoint *nextMergeNode;
        SteinerPoint *nodeBeforeNextMergeNode = curNode; // could be merge node, snake node, TSV node or L shape node after TSV insertion
        SteinerPoint *curNodeParent = curNode->parent;

        while (!(nodeBeforeNextMergeNode->parent->actualMergeNode))
        {
            nodeBeforeNextMergeNode = nodeBeforeNextMergeNode->parent;
        }
        nextMergeNode = nodeBeforeNextMergeNode->parent;
        assert(nextMergeNode->actualMergeNode);

        if (curNode->buffered)
        {
            if (addedBuffer.find(steinerPointToIdMap[curNode]) == addedBuffer.end()) // buffer not inserted yet
            {
                //! two buffers(inverters) so the signal is not inverted
                Point_2D firstBufferNode;
                int firstBufferNodeName = nodeName;
                firstBufferNode.x = curNode->x;
                firstBufferNode.y = curNode->y;
                bufferNodes.push_back(make_pair(firstBufferNodeName, firstBufferNode));
                nodeName++;

                Point_2D secondBufferNode;
                int secondBufferNodeName = nodeName;
                secondBufferNode = firstBufferNode;
                bufferNodes.push_back(make_pair(secondBufferNodeName, secondBufferNode));
                nodeName++;

                // curNode <- buffer1 <- buffer2 <- curNodeParent
                //! be aware of wire direction
                wire wire1;
                wire1.leftId = firstBufferNodeName;
                wire1.rightId = steinerPointToIdMap[curNode];
                wire1.metalIndex = 0; // !!!!this wire is not inverter!!!!

                wire wire2;
                wire2.leftId = secondBufferNodeName;
                wire2.rightId = firstBufferNodeName;
                wire2.metalIndex = 0; // 0 for all buffers

                wire wire3;
                wire3.leftId = steinerPointToIdMap[curNodeParent];
                wire3.rightId = secondBufferNodeName;
                wire3.metalIndex = nextMergeNode->metalLayerIndex; // 0 for all buffers

                bufferWires.push_back(wire1);
                bufferWires.push_back(wire2);
                wires.push_back(wire3);

                addedBuffer.insert(steinerPointToIdMap[curNode]);
            }
        }

        traceToSource_addBuffers(nextMergeNode);
    };
    for (int tapId = 0; tapId < db->dbSinks.size(); tapId++)
    {
        traceToSource_addBuffers(solution[tapId]);
    }

    ///////////////////////////////////////////////////////
    // !Step 4: add all other wires(including TSV wires)
    ///////////////////////////////////////////////////////
    std::function<void(SteinerPoint *)> traceToSource_addWires = [&](SteinerPoint *curNode)
    {
        if (curNode->parent == NULL)
        { // reached source
            return;
        }

        assert(curNode->actualMergeNode);

        SteinerPoint *nextMergeNode;
        SteinerPoint *nodeBeforeNextMergeNode = curNode; // could be merge node, snake node, TSV node or L shape node after TSV insertion

        while (!(nodeBeforeNextMergeNode->parent->actualMergeNode))
        {
            nodeBeforeNextMergeNode = nodeBeforeNextMergeNode->parent;
        }
        nextMergeNode = nodeBeforeNextMergeNode->parent;

        assert(nextMergeNode->actualMergeNode);
        if (addedWire.find(steinerPointToIdMap[curNode]) == addedWire.end()) //! if wire between these 2 ndoes are not added yet
        {
            if (curNode->buffered)
            {
                if (curNode->layer == nextMergeNode->layer)
                {
                    // no TSV but buffered
                    //! 3 scenarios:
                    //! 1.curNode->BUFFER1->BUFFER2->L-shapeNode->nextMergeNode
                    //! 2.curNode->BUFFER1->BUFFER2->snakeNode->snakeNode->nextMergeNode
                    //! 3.curNode->BUFFER1->BUFFER2->nextMergeNode
                    SteinerPoint *wireRight = curNode->parent;
                    SteinerPoint *wireLeft = wireRight->parent;
                    wire tempWire;
                    while (wireRight != nextMergeNode)
                    {
                        tempWire.leftId = steinerPointToIdMap[wireLeft];
                        tempWire.rightId = steinerPointToIdMap[wireRight];
                        tempWire.metalIndex = wireLeft->metalLayerIndex;
                        wires.push_back(tempWire);
                        wireRight = wireRight->parent;
                        wireLeft = wireLeft->parent;
                    }
                }
                else
                {
                    // buffered and there is TSV
                    // TSV->parent = nextMergeNode
                    assert(nodeBeforeNextMergeNode->isTSVNode);
                    //! 3 scenarios:
                    //! 1.curNode->BUFFER1->BUFFER2->L-shapeNode->TSV->nextMergeNode
                    //! 2.curNode->BUFFER1->BUFFER2->snakeNode->snakeNode->TSV->nextMergeNode
                    //! 3.curNode->BUFFER1->BUFFER2->TSV->nextMergeNode
                    SteinerPoint *wireRight = curNode->parent;
                    SteinerPoint *wireLeft = wireRight->parent;
                    wire tempWire;
                    while (wireRight != nextMergeNode)
                    {
                        tempWire.leftId = steinerPointToIdMap[wireLeft];
                        tempWire.rightId = steinerPointToIdMap[wireRight];
                        tempWire.metalIndex = wireLeft->metalLayerIndex;
                        wires.push_back(tempWire);
                        wireRight = wireRight->parent;
                        wireLeft = wireLeft->parent;
                    }
                }
            }
            else
            {
                if (curNode->layer == nextMergeNode->layer)
                {
                    // no TSV and no buffer
                    //! 3 scenarios:
                    //! 1.curNode->L-shapeNode->nextMergeNode
                    //! 2.curNode->snakeNode->snakeNode->nextMergeNode
                    //! 3.curNode->nextMergeNode
                    SteinerPoint *wireRight = curNode;
                    SteinerPoint *wireLeft = wireRight->parent;
                    wire tempWire;

                    while (wireRight != nextMergeNode)
                    {
                        tempWire.leftId = steinerPointToIdMap[wireLeft];
                        tempWire.rightId = steinerPointToIdMap[wireRight];
                        tempWire.metalIndex = wireLeft->metalLayerIndex;
                        wires.push_back(tempWire);
                        wireRight = wireRight->parent;
                        wireLeft = wireLeft->parent;
                    }
                }
                else
                {
                    // no buffer but there is TSV
                    //! 3 scenarios:
                    //! 1.curNode->L-shapeNode->TSV->nextMergeNode
                    //! 2.curNode->snakeNode->snakeNode->TSV->nextMergeNode
                    //! 3.curNode->TSV->nextMergeNode
                    // TSV->parent = nextMergeNode
                    assert(nodeBeforeNextMergeNode->isTSVNode);
                    SteinerPoint *wireRight = curNode;
                    SteinerPoint *wireLeft = wireRight->parent;
                    wire tempWire;
                    while (wireRight != nextMergeNode)
                    {
                        tempWire.leftId = steinerPointToIdMap[wireLeft];
                        tempWire.rightId = steinerPointToIdMap[wireRight];
                        tempWire.metalIndex = wireLeft->metalLayerIndex;
                        wires.push_back(tempWire);
                        wireRight = wireRight->parent;
                        wireLeft = wireLeft->parent;
                    }
                }
            }
        }
        addedWire.insert(steinerPointToIdMap[curNode]);
        traceToSource_addWires(nextMergeNode);
    };
    for (int tapId = 0; tapId < db->dbSinks.size(); tapId++)
    {
        traceToSource_addWires(solution[tapId]);
    }

    ///////////////////////////////////////////////////////
    // !Step 5: add source buffers and that one inverter, and buffer for root, and all these buffer wires
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // !Step 6: output
    ///////////////////////////////////////////////////////tream fout(setting.output_file_name+"/"+setting.get_case_name()+"_script");
    assert(topology->root->buffered);
    string benchmarkName;
    string outputPath;
    if (!gArg.GetString("output", &outputPath))
    {
        outputPath = "./";
    }
    string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    gArg.GetString("benchmarkName", &benchmarkName);

    ofstream fout(outputPath + "/" + benchmarkName + "_script");
    fout.setf(ios::fixed, ios::floatfield);
    if (fout.fail())
    {
        cout << "Fail to open file:" << outputPath + "/" + benchmarkName + "_script" << endl;
        exit(1);
    }
    else
    {
        cout << padding << "Output to:" << outputPath + "/" + benchmarkName + "_script" << padding << endl;
    }
    fout << "sourcenode 0 0\n";
    // fout << "num node " << nodeName - 1 << endl;
    // for (int i = 1; i <= nonBufferNodeCount; i++)
    // {
    //     fout << i << " " << IDtoSteinerPointMap[i]->x << " " << IDtoSteinerPointMap[i]->y;
    //     fout << temp.id << " " << temp.x << " " << temp.y << endl;
    // }

    fout << "num node " << nodeName - 1 << endl;
    fout << "# non-buffer nodes" << endl;

    for (int nodeId = db->dbSinks.size() + 1; nodeId <= nonBufferNodeCount; nodeId++)
    {
        // fout << steinerPointToIdMap[solution[sinkId]]<<" "<<sinkId+1 << " " << solution[sinkId]->x << " " << solution[sinkId]->y << endl;
        assert(IDtoSteinerPointMap.find(nodeId) != IDtoSteinerPointMap.end());
        fout << nodeId << " " << IDtoSteinerPointMap[nodeId]->x << " " << IDtoSteinerPointMap[nodeId]->y << endl;
        if(IDtoSteinerPointMap[nodeId]->x<0.0||IDtoSteinerPointMap[nodeId]->y<0.0)
        {
            cout<<IDtoSteinerPointMap[nodeId]->actualMergeNode<<" "<<IDtoSteinerPointMap[nodeId]->isTSVNode<<" "<<IDtoSteinerPointMap[nodeId]->snakeNode<<endl;
        }
    }

    fout << "# buffer nodes" << endl;
    for (pair<int, Point_2D> curBufferNode : bufferNodes)
    {
        fout << curBufferNode.first << " " << curBufferNode.second.x << " " << curBufferNode.second.y << endl;
    }
    fout << "num sinknode " << db->dbSinks.size() << endl;
    for (int sinkId = 0; sinkId < db->dbSinks.size(); sinkId++)
    {
        // fout << steinerPointToIdMap[solution[sinkId]]<<" "<<sinkId+1 << " " << solution[sinkId]->x << " " << solution[sinkId]->y << endl;
        fout << steinerPointToIdMap[solution[sinkId]] << " " << sinkId + 1 << endl;
    }

    // fout << "num wire " << wires.size() + source_root_wires.size() << endl;
    // for (wire temp : source_root_wires)
    // {
    //     fout << temp.left_id << " " << temp.right_id << " " << temp.metal_index << endl;
    // }
    // for (wire tempwire : wires)
    // {
    //     fout << tempwire.left_id << " " << tempwire.right_id << " " << tempwire.metal_index << endl;
    // }

    fout << "num buffer " << bufferWires.size() << endl;
    for (wire temp : bufferWires)
    {
        fout << temp.leftId << " " << temp.rightId << " " << temp.metalIndex << endl;
    }
    // for (wire tempwire : buffers)
    // {
    //     fout << tempwire.left_id << " " << tempwire.right_id << " " << tempwire.metal_index << endl;
    // }

    //! count TSV, buffer and wirelength

    std::function<void(TreeNode *)> preOrderTraversal_Count = [&](TreeNode *curNode)
    {
        if (curNode->leftChild != NULL && curNode->rightChild != NULL)
        {
            cout << "current layer: " << curNode->layer << endl;
            if (curNode->leftChild->layer != curNode->layer)
            {
                TSVCount++;
            }
            if (curNode->rightChild->layer != curNode->layer)
            {
                TSVCount++;
            }
            if (curNode->buffered)
            {
                bufferCount++;
            }
            preOrderTraversal_Count(curNode->leftChild);
            preOrderTraversal_Count(curNode->rightChild);
        }
        else
        {
            // reached sink
        }
    };
    preOrderTraversal_Count(topology->root);
    fout << "# root at " << *solution[topology->root->id] << endl
         << "# buffer count: " << bufferCount << endl
         << "# TSV count: " << TSVCount << endl;
}

void ZSTDMERouter::metalLayerAssignment()
{
    //! assume metal layer index start from 0, no dummy metal now!
    int treeLevelCount = 0;                                                    // number of tree levels
    int metalLayerCount = db->dbMetals.size();                                 //! no dummy metal!!!
    std::function<void(TreeNode *)> preOrderTraversal = [&](TreeNode *curNode) //!&
    {
        if (curNode)
        {
            int curId = curNode->id;
            if (curNode->parent)
            {
                curNode->level = curNode->parent->level + 1;
                if (curNode->level > treeLevelCount)
                {
                    treeLevelCount = curNode->level;
                }
            }
            preOrderTraversal(curNode->leftChild);
            preOrderTraversal(curNode->rightChild);
        }
    };
    topology->root->level = 1; // tree layer start from 1 rather than 0
    assert(!topology->root->parent);
    preOrderTraversal(topology->root);

    std::function<void(TreeNode *)> preOrderTraversal_SetMetal = [&](TreeNode *curNode)
    {
        if (curNode)
        {
            int curId = curNode->id;
            curNode->metalLayerIndex = ceil((double(curNode->level) / double(treeLevelCount)) * double(db->dbMetals.size())) - 1; // -1 because index of the vector dbMetals starts from 0
            assert(curNode->metalLayerIndex < db->dbMetals.size());                                                               //!
            preOrderTraversal_SetMetal(curNode->leftChild);
            preOrderTraversal_SetMetal(curNode->rightChild);
        }
    };
    preOrderTraversal_SetMetal(topology->root);
}

void ZSTDMERouter::DLE_3D()
{
    DLE_loop(topology->root);
    topology->root->layer = topology->root->el.first;
    assert(topology->root->parent == NULL);
    NearestAssign(topology->root);
}

void ZSTDMERouter::DLE_loop(TreeNode *node)
{
    if (node)
    {
        // 如果已经是根节点，跳过
        if (!node->leftChild && !node->rightChild)
        {
            node->el.first = node->layer;
            node->el.second = node->layer;
            return;
        }
        // 自下而上递归
        DLE_loop(node->leftChild);
        DLE_loop(node->rightChild);

        int l1 = max(node->leftChild->el.first, node->rightChild->el.first);
        int l2 = min(node->leftChild->el.second, node->rightChild->el.second);
        node->el.first = min(l1, l2);
        node->el.second = max(l1, l2);
    }
}

void ZSTDMERouter::NearestAssign(TreeNode *node)
{
    if (!node->leftChild && !node->rightChild)
        return;

    if (node->parent)
    {
        assert(node->el.first <= node->el.second);
        if (node->el.first > node->parent->layer)
        {
            node->layer = node->el.first;
        }
        else if (node->el.second < node->parent->layer)
        {
            node->layer = node->el.second;
        }
        else
        {
            node->layer = node->parent->layer;
        }
    }

    NearestAssign(node->leftChild);
    NearestAssign(node->rightChild);
}

SteinerPoint *ZSTDMERouter::insertTSVNode(SteinerPoint *curNode, set<pair<int, int>> &addedTSV)
{
    // assert(curNode->actualMergeNode && !curNode->snakeNode);
    // assert(curNode->parent);
    // SteinerPoint *mergeNode;
    // SteinerPoint *TSV;

    // if (curNode->parent->snakeNode) // snaking, and there must be 2 snaking nodes
    // {
    //     assert(curNode->parent->parent->snakeNode);
    //     // cur -> snakenode1 -> snakenode2 -> mergenode
    //     SteinerPoint *snakeNodeBeforeNextMergeNode = curNode->parent->parent;
    //     mergeNode = snakeNodeBeforeNextMergeNode->parent;
    //     assert(mergeNode->actualMergeNode);

    //     if (curNode->layer == mergeNode->layer ||)
    //     {
    //         return mergeNode;
    //     }

    //     Point_2D TSVnodeLocation = determineTSVLocation(snakeNodeBeforeNextMergeNode, mergeNode);

    //     TSV = new SteinerPoint(TSVnodeLocation);

    //     snakeNodeBeforeNextMergeNode->parent = TSV;

    //     TSV->isTSVNode = true;
    //     TSV->metalLayerIndex = mergeNode->metalLayerIndex;
    //     TSV->parent = mergeNode;
    // }
    // else if (curNode->parent->actualMergeNode)
    // {
    //     // cur -> mergenode
    //     mergeNode = curNode->parent;
    //     assert(mergeNode->actualMergeNode);
    //     if (curNode->layer == mergeNode->layer)
    //     {
    //         return mergeNode;
    //     }

    //     Point_2D TSVnodeLocation = determineTSVLocation(curNode, mergeNode);

    //     TSV = new SteinerPoint(Point_2D(TSVnodeLocation));

    //     curNode->parent = TSV;

    //     TSV->isTSVNode = true;
    //     TSV->metalLayerIndex = mergeNode->metalLayerIndex;
    //     TSV->parent = mergeNode;
    // }
    // else // no snaking, just L shape
    // {
    //     // cur -> middleNodeForLshape -> mergenode
    //     assert(curNode->parent->parent->actualMergeNode);
    //     SteinerPoint *middleNode = curNode->parent;
    //     mergeNode = curNode->parent->parent;
    //     assert(mergeNode->actualMergeNode);
    //     if (curNode->layer == mergeNode->layer)
    //     {
    //         return mergeNode;
    //     }

    //     Point_2D TSVnodeLocation = determineTSVLocation(middleNode, mergeNode);

    //     TSV = new SteinerPoint(Point_2D(TSVnodeLocation));

    //     middleNode->parent = TSV;

    //     TSV->isTSVNode = true;
    //     TSV->metalLayerIndex = mergeNode->metalLayerIndex;
    //     TSV->parent = mergeNode;
    // }
    // return TSV;
}

Point_2D ZSTDMERouter::determineTSVLocation(SteinerPoint *before, SteinerPoint *merge)
{
    // before->TSV->merge
    // the before node must have same x or y with merge
    assert(double_equal(before->x, merge->x) || double_equal(before->y, merge->y));
    Point_2D TSVlocation;
    if (double_equal(before->x, merge->x))
    {
        TSVlocation.x = before->x;
        if (double_less(before->y, merge->y))
        {
            TSVlocation.y = merge->y - 1.0;
        }
        else
        {
            TSVlocation.y = merge->y + 1.0;
        }
    }
    else
    {
        TSVlocation.y = before->y;
        if (double_less(before->x, merge->x))
        {
            TSVlocation.x = merge->x - 1.0;
        }
        else
        {
            TSVlocation.x = merge->x + 1.0;
        }
    }
    return TSVlocation;
}

// Segment ZSTDMERouter::TRRintersectTRR(TRR &trr1, TRR &trr2)
// {
//     // get four edges
//     // cout << "Merging: " << trr1 << " and " << trr2 << endl;
//     vector<Point_2D> trr1_boundary_grid;
//     vector<Point_2D> trr2_boundary_grid;
//     vector<Segment> trr1_Sides;
//     vector<Segment> trr2_Sides;
//     assert(trr1.radius != 0 || trr2.radius != 0);

//     //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     // todo: what if a core is a leaf and its radius is 0? judge if its in a trr, but we didn't run into this case?
//     //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//     //! if there is one trr's radius = 0
//     if (trr1.radius == 0 || trr2.radius == 0)
//     {
//         if (trr1.radius == 0)
//         {

//             if (trr2.core.slope() > 0)
//             {
//                 trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x, trr2.core.lowerPoint.y - trr2.radius);
//                 trr2_boundary_grid.emplace_back(trr2.core.higherPoint.x + trr2.radius, trr2.core.higherPoint.y);
//                 trr2_boundary_grid.emplace_back(trr2.core.higherPoint.x, trr2.core.higherPoint.y + trr2.radius);
//                 trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x - trr2.radius, trr2.core.lowerPoint.y); // clock-wise
//             }
//             else if (trr2.core.slope() < 0)
//             {
//                 trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x + trr2.radius, trr2.core.lowerPoint.y);
//                 trr2_boundary_grid.emplace_back(trr2.core.higherPoint.x, trr2.core.higherPoint.y + trr2.radius);
//                 trr2_boundary_grid.emplace_back(trr2.core.higherPoint.x - trr2.radius, trr2.core.higherPoint.y);
//                 trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x, trr2.core.lowerPoint.y - trr2.radius); // clock-wise
//             }
//             else
//             { // leaf node
//                 trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x, trr2.core.lowerPoint.y - trr2.radius);
//                 trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x + trr2.radius, trr2.core.lowerPoint.y);
//                 trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x, trr2.core.lowerPoint.y + trr2.radius);
//                 trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x - trr2.radius, trr2.core.lowerPoint.y); // clock-wise
//             }

//             for (int i = 0; i < 3; i++)
//             {
//                 trr2_Sides.emplace_back(trr2_boundary_grid[i], trr2_boundary_grid[i + 1]);
//             }
//             trr2_Sides.emplace_back(trr2_boundary_grid[3], trr2_boundary_grid[0]);

//             for (auto &seg2 : trr2_Sides)
//             {
//                 // cout<<"seg1: "<<seg1<<"seg2: "<<seg2<<endl;
//                 Segment seg = trr1.core.intersect(seg2); //! seg should be a single point in most cases
//                 // ? could there be 2 intersection points for core and trr sides?
//                 if (seg.id == 0)
//                 {
//                     return seg;
//                 }
//                 else if (seg.id == 1) // single point intersection
//                 {
//                     if (trr2.insideTRR(trr1.core.lowerPoint))
//                     {
//                         return Segment(seg.lowerPoint, trr1.core.lowerPoint);
//                     }
//                     else if (trr2.insideTRR(trr1.core.higherPoint))
//                     {
//                         return Segment(seg.lowerPoint, trr1.core.higherPoint);
//                     }
//                 }
//             }
//             //! if trr1.core is completely inside trr2
//             if (trr2.insideTRR(trr1.core.lowerPoint) && trr2.insideTRR(trr1.core.higherPoint))
//             {
//                 Segment trr1core(trr1.core);
//                 trr1core.id = 0;
//                 return trr1core;
//             }
//             cout << endl
//                  << trr2.insideTRR(trr1.core.lowerPoint) << " ff " << trr2.insideTRR(trr1.core.higherPoint) << endl;
//         }
//         else
//         {
//             if (trr1.core.slope() > 0)
//             {
//                 trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x, trr1.core.lowerPoint.y - trr1.radius);
//                 trr1_boundary_grid.emplace_back(trr1.core.higherPoint.x + trr1.radius, trr1.core.higherPoint.y);
//                 trr1_boundary_grid.emplace_back(trr1.core.higherPoint.x, trr1.core.higherPoint.y + trr1.radius);
//                 trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x - trr1.radius, trr1.core.lowerPoint.y); // clock-wise
//             }
//             else if (trr1.core.slope() < 0)
//             {
//                 trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x + trr1.radius, trr1.core.lowerPoint.y);
//                 trr1_boundary_grid.emplace_back(trr1.core.higherPoint.x, trr1.core.higherPoint.y + trr1.radius);
//                 trr1_boundary_grid.emplace_back(trr1.core.higherPoint.x - trr1.radius, trr1.core.higherPoint.y);
//                 trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x, trr1.core.lowerPoint.y - trr1.radius); // clock-wise
//             }
//             else
//             { // leaf node
//                 trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x, trr1.core.lowerPoint.y - trr1.radius);
//                 trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x + trr1.radius, trr1.core.lowerPoint.y);
//                 trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x, trr1.core.lowerPoint.y + trr1.radius);
//                 trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x - trr1.radius, trr1.core.lowerPoint.y); // clock-wise
//             }

//             for (int i = 0; i < 3; i++)
//             {
//                 trr1_Sides.emplace_back(trr1_boundary_grid[i], trr1_boundary_grid[i + 1]);
//             }
//             trr1_Sides.emplace_back(trr1_boundary_grid[3], trr1_boundary_grid[0]);

//             for (auto &seg1 : trr1_Sides)
//             {
//                 // cout<<"seg1: "<<seg1<<"seg2: "<<seg2<<endl;
//                 Segment seg = trr2.core.intersect(seg1); //! seg should be a single point in most cases
//                 // ? could there be 2 intersection points for core and trr sides?
//                 if (seg.id == 0)
//                 {
//                     return seg;
//                 }
//                 else if (seg.id == 1) // return single point
//                 {
//                     if (trr1.insideTRR(trr2.core.lowerPoint))
//                     {
//                         return Segment(seg.lowerPoint, trr2.core.lowerPoint);
//                     }
//                     else if (trr1.insideTRR(trr2.core.higherPoint))
//                     {
//                         return Segment(seg.lowerPoint, trr2.core.higherPoint);
//                     }
//                 }
//             }
//             if (trr1.insideTRR(trr2.core.lowerPoint) && trr1.insideTRR(trr2.core.higherPoint))
//             {
//                 Segment trr2core(trr2.core);
//                 trr2core.id = 0;
//                 return trr2core;
//             }
//             cout << endl
//                  << trr1.insideTRR(trr2.core.lowerPoint) << " gg " << trr1.insideTRR(trr2.core.higherPoint) << endl;
//         }

//         cout << "Cannot find intersection between two TRRs when one TRR has radius == 0" << endl;
//         for (auto &seg1 : trr1_Sides)
//         {
//             for (auto &seg2 : trr2_Sides)
//             {
//                 if (double_equal(seg1.slope(), seg2.slope()))
//                 {
//                     cout << "equal slope: " << endl;
//                     // check if 4 point same line but rejected due to precision problems
//                     cout << (seg2.lowerPoint.y - seg1.lowerPoint.y) * (seg1.higherPoint.x - seg1.lowerPoint.x) << " " << (seg1.higherPoint.y - seg1.lowerPoint.y) * (seg2.lowerPoint.x - seg1.lowerPoint.x);
//                 }
//             }
//         }
//         drawTRRPair("bottomup_debug", trr1, trr2);
//         Segment ret;
//         ret.id = -1;
//         return ret;
//     }
//     // cout<<"radius check*******888\n";
//     //  if both trr's radius > 0
//     if (trr1.core.slope() > 0)
//     {
//         trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x, trr1.core.lowerPoint.y - trr1.radius);
//         trr1_boundary_grid.emplace_back(trr1.core.higherPoint.x + trr1.radius, trr1.core.higherPoint.y);
//         trr1_boundary_grid.emplace_back(trr1.core.higherPoint.x, trr1.core.higherPoint.y + trr1.radius);
//         trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x - trr1.radius, trr1.core.lowerPoint.y); // clock-wise
//     }
//     else if (trr1.core.slope() < 0)
//     {
//         trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x + trr1.radius, trr1.core.lowerPoint.y);
//         trr1_boundary_grid.emplace_back(trr1.core.higherPoint.x, trr1.core.higherPoint.y + trr1.radius);
//         trr1_boundary_grid.emplace_back(trr1.core.higherPoint.x - trr1.radius, trr1.core.higherPoint.y);
//         trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x, trr1.core.lowerPoint.y - trr1.radius); // clock-wise
//     }
//     else
//     { // leaf node
//         trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x, trr1.core.lowerPoint.y - trr1.radius);
//         trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x + trr1.radius, trr1.core.lowerPoint.y);
//         trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x, trr1.core.lowerPoint.y + trr1.radius);
//         trr1_boundary_grid.emplace_back(trr1.core.lowerPoint.x - trr1.radius, trr1.core.lowerPoint.y); // clock-wise
//     }

//     if (trr2.core.slope() > 0)
//     {
//         trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x, trr2.core.lowerPoint.y - trr2.radius);
//         trr2_boundary_grid.emplace_back(trr2.core.higherPoint.x + trr2.radius, trr2.core.higherPoint.y);
//         trr2_boundary_grid.emplace_back(trr2.core.higherPoint.x, trr2.core.higherPoint.y + trr2.radius);
//         trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x - trr2.radius, trr2.core.lowerPoint.y); // clock-wise
//     }
//     else if (trr2.core.slope() < 0)
//     {
//         trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x + trr2.radius, trr2.core.lowerPoint.y);
//         trr2_boundary_grid.emplace_back(trr2.core.higherPoint.x, trr2.core.higherPoint.y + trr2.radius);
//         trr2_boundary_grid.emplace_back(trr2.core.higherPoint.x - trr2.radius, trr2.core.higherPoint.y);
//         trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x, trr2.core.lowerPoint.y - trr2.radius); // clock-wise
//     }
//     else
//     { // leaf node
//         trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x, trr2.core.lowerPoint.y - trr2.radius);
//         trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x + trr2.radius, trr2.core.lowerPoint.y);
//         trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x, trr2.core.lowerPoint.y + trr2.radius);
//         trr2_boundary_grid.emplace_back(trr2.core.lowerPoint.x - trr2.radius, trr2.core.lowerPoint.y); // clock-wise
//     }

//     for (int i = 0; i < 3; i++)
//     {
//         trr1_Sides.emplace_back(trr1_boundary_grid[i], trr1_boundary_grid[i + 1]);
//         trr2_Sides.emplace_back(trr2_boundary_grid[i], trr2_boundary_grid[i + 1]);
//     }
//     trr1_Sides.emplace_back(trr1_boundary_grid[3], trr1_boundary_grid[0]);
//     trr2_Sides.emplace_back(trr2_boundary_grid[3], trr2_boundary_grid[0]);

//     // cout << "Print trr1's sides" << endl;
//     // for (auto& seg1 : trr1_Sides) {
//     //     cout << seg1 << endl;
//     // }

//     // cout << "Print trr2's sides" << endl;

//     // for (auto& seg2 : trr2_Sides) {
//     //     cout << seg2 << endl;
//     // }

//     // for 4*4 check intersect
//     Segment tempseg; // we need this to prevent one scenario:
//     // 当两个TRR有边重合（斜率相等），令斜率相等的两边为TRR1.x和TRR2.x，那么，对于TRR1，其中当然还有边TRR1.y与TRR1.x垂直。那么这个循环有可能先返回TRR1.y与TRR2.x的交点！导致错误
//     // 为此，当获得point intersection时，不急着返回，再等等看有没有segment
//     // intersection
//     tempseg.id = -1;
//     for (auto &seg1 : trr1_Sides)
//     {
//         for (auto &seg2 : trr2_Sides)
//         {
//             Segment seg = seg1.intersect(seg2);
//             if (seg.id == 0)
//             {
//                 return seg;
//             }
//             if (seg.id == 1)
//             {
//                 tempseg = seg;
//             }
//         }
//     }
//     if (tempseg.id == 1)
//     {
//         return tempseg;
//     }

//     cout << "Cannot find intersection between two TRRs" << endl;

//     for (auto &seg1 : trr1_Sides)
//     {
//         for (auto &seg2 : trr2_Sides)
//         {
//             if (double_equal(seg1.slope(), seg2.slope()))
//             {
//                 cout << "equal slope: " << endl;
//                 // check if 4 point same line but rejected due to precision problems
//                 cout << (seg2.lowerPoint.y - seg1.lowerPoint.y) * (seg1.higherPoint.x - seg1.lowerPoint.x) << " " << (seg1.higherPoint.y - seg1.lowerPoint.y) * (seg2.lowerPoint.x - seg1.lowerPoint.x) << endl;
//             }
//         }
//     }
//     drawTRRPair("bottomup_debug", trr1, trr2);
//     Segment ret;
//     ret.id = -1;
//     return ret;
// }

// void ZSTDMERouter::updateMergeCapacitance(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb)
// {

//     double mergeCapacitance = nodeLeft->loadCapacitance + nodeRight->loadCapacitance + UNIT_CAPACITANCE * (ea + eb);
//     // 考虑了buffer insertion的电容update
//     //  if(delta_C > c_constraint){
//     //      nodeMerge->load_capacitance = 300;
//     //      nodeMerge->buffered=true;
//     //      buffercount++;
//     //      //nodeMerge->needBuffer = 1;
//     //      return;
//     //  }
//     nodeMerge->loadCapacitance = mergeCapacitance;
// }

// void ZSTDMERouter::updateMergeDelay(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb)
// {
//     double delayToLeft;
//     double delayToRight;

//     if (delayModel == LINEAR_DELAY)
//     {
//         delayToLeft = nodeLeft->delay + ea;
//         delayToRight = nodeRight->delay + eb;
//     }
//     else if (delayModel == ELMORE_DELAY) // refer to the abk paper for this
//     {
//         delayToLeft = nodeLeft->delay + 0.5 * UNIT_RESISTANCE * UNIT_CAPACITANCE * ea * ea +
//                       UNIT_RESISTANCE * nodeLeft->loadCapacitance * ea;
//         delayToRight = nodeRight->delay + 0.5 * UNIT_RESISTANCE * UNIT_CAPACITANCE * eb * eb +
//                        UNIT_RESISTANCE * nodeRight->loadCapacitance * eb;
//     }

//     nodeMerge->delay = max(delayToLeft, delayToRight);
// }

// double ZSTDMERouter::solveForX(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, double L) // refer to the abk paper for this
// {
//     // x is the x in abk paper, and ea=x, eb=L-x,
//     double numerator = (nodeRight->delay - nodeLeft->delay) + UNIT_RESISTANCE * L * (nodeRight->loadCapacitance + 0.5 * UNIT_CAPACITANCE * L);
//     double denominator = UNIT_RESISTANCE * (nodeLeft->loadCapacitance + nodeRight->loadCapacitance + UNIT_CAPACITANCE * L);
//     double x = numerator / denominator;
//     return x;
// }

// double ZSTDMERouter::solveForLPrime(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, int tag)
// {

//     // tag = 0: |eb| = L'
//     // tag = 1: |ea| = L'
//     double alphaC;
//     double numerator;
//     if (tag == 0)
//     {
//         alphaC = UNIT_RESISTANCE * nodeRight->loadCapacitance; // see abk paper
//         numerator = sqrt(2 * UNIT_RESISTANCE * UNIT_CAPACITANCE * (nodeLeft->delay - nodeRight->delay) + alphaC * alphaC - 2 * UNIT_RESISTANCE * UNIT_CAPACITANCE) - alphaC;
//     }
//     else
//     {
//         alphaC = UNIT_RESISTANCE * nodeLeft->loadCapacitance;
//         numerator = sqrt(2 * UNIT_RESISTANCE * UNIT_CAPACITANCE * (nodeRight->delay - nodeLeft->delay) + alphaC * alphaC - 2 * UNIT_RESISTANCE * UNIT_CAPACITANCE) - alphaC;
//     }
//     return numerator / (UNIT_RESISTANCE * UNIT_CAPACITANCE);
// }

Segment ZSTDMERouter::nineRegionBasedFeasibleMergeSegmentCutting(Segment merge, Segment child)
{
    double minimumDistance = minManhattanDist(merge, child);

    vector<int> mergeSegRegion;
    vector<int> childSegRegion;

    Segment feasibleMergeSegment = merge;

    for (Blockage curBlockage : db->dbBlockages)
    {
        vector<Segment> mergeCutByBlockage = segmentCutByBlockage(feasibleMergeSegment, curBlockage);
        vector<Segment> childCutByBlockage = segmentCutByBlockage(child, curBlockage);
        // determine the region of each resulted segment
        //? 如果在九区法筛选之后，mergeSegment 变成好几段（共线但无交点）该怎么办？有这种可能：当segment从blcokage中间穿过时，处于MID-MID region的那一段肯定会被去掉
        // 一种办法：主动放弃至只剩一段？会不会导致找不到可行解？不会。但应该可能导致可避免的detour（用这段segment就需要detour，那段就不需要），有可能影响解的质量（线长），如果随机放弃某一段的话，那么既不会得到最坏解也不会得到最优解
        // detour 影响的是时钟树总线长，不影响skew
        // todo：决定策略（算法细节）

        //!! 如果childSegment到blockage1边缘上的detour点的路被blockage2挡了咋办？咋选detour点？ 不overlap比skew更重要？
        //!! 这种情况：对线长和skew有影响，但肯定可以做到不overlap（blockage不会接触或重叠），只要发生detour，其他的blockage可以都不管了，都在走线过程中绕就好了，只要在具体走线过程中计算好线长就行，那么怎么计算呢？？？
        //!! 或者只要有detour，就综合所有blockage来考虑ms的位置？
        for (Segment curSubMergeSeg : mergeCutByBlockage)
        {
            mergeSegRegion.push_back(regionNumber(curSubMergeSeg, curBlockage));
        }
        for (Segment curSubChildSeg : childCutByBlockage)
        {
            childSegRegion.push_back(regionNumber(curSubChildSeg, curBlockage));
        }
    }
    return Segment();
}