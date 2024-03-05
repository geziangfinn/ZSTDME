#include "topology.h"

void TreeTopology::buildTopoUsingNearestNeighborGraph()
{
    vector<TreeNode *> treeNodes;
    vector<NngNode> NngNodes;
    int index = 0;
    for (Sink curSink : db->dbSinks)
    {
        NngNode node;
        assert(index == curSink.id);

        node.x = curSink.x;
        node.y = curSink.y;
        node.loadCapacitance = curSink.capacitance;

        TreeNode *curRoot = new TreeNode(curSink.id);
        curRoot->loadCapacitance = curSink.capacitance;

        treeNodes.emplace_back(curRoot);
        NngNodes.emplace_back(node);
        index++;
    }
    int leafCount = db->dbSinks.size();
    cout << "leafCount: " << leafCount << endl;
    // treeNodes is the S, after every merge merged, the merge node is added to S
    vector<NngNodePair> currentMinimumPairs;

    vector<bool> merged;
    merged.resize(treeNodes.size(), false);

    int unmergedNodeCount = treeNodes.size();

    while (unmergedNodeCount > 1) //! this while loop surely is the bottleneck for nng calculation. A potential "contribution"?
    {
        // calculate new nng and pick one pair to merge
        for (int i = 0; i < treeNodes.size(); i++)
        {
            assert(treeNodes[i]->id == i);
            if (!merged[i])
            {
                double cost = numeric_limits<double>::max();
                int to = -1;
                for (int j = 0; j < treeNodes.size(); j++)
                {
                    if ((i != j) && !merged[j])
                    {
                        if (nearestNeighborPairCost(NngNodes[i], NngNodes[j]) < cost)
                        {
                            cost = nearestNeighborPairCost(NngNodes[i], NngNodes[j]);
                            to = j;
                        }
                    }
                }
                assert(to != -1);
                currentMinimumPairs.emplace_back(NngNodePair(treeNodes[i]->id, treeNodes[to]->id, cost)); //! treeNodes[i]->id==i, treeNodes[to]->id==to
            }
        }
        //! find a pair with the smallest cost in currentMinimumPairs and merge
        sort(currentMinimumPairs.begin(), currentMinimumPairs.end(), [=](NngNodePair a, NngNodePair b)
             {
                 return a.cost <= b.cost; //! pair with smaller cost are placed at front
             });                          //? sort to find the smallest is a waste?(nlogn instead of n)

        NngNodePair best_pair = currentMinimumPairs.front();

        TreeNode *mergeNode = new TreeNode(treeNodes.size()); //!

        double mergedX = (NngNodes[best_pair.from].x + NngNodes[best_pair.to].x) / 2;
        double mergedY = (NngNodes[best_pair.from].y + NngNodes[best_pair.to].y) / 2;
        double mergedCapacitance = NngNodes[best_pair.from].loadCapacitance + NngNodes[best_pair.to].loadCapacitance;
        // capacitance of treeNodes are updated in DME

        NngNode mergedNngNode(mergedX, mergedY, mergedCapacitance);

        mergeNode->leftChild = treeNodes[best_pair.from];
        mergeNode->rightChild = treeNodes[best_pair.to];

        treeNodes[best_pair.from]->parent = mergeNode;
        treeNodes[best_pair.to]->parent = mergeNode;

        treeNodes.emplace_back(mergeNode); // mergeNode->id == its index in the vector treeNodes!
        unmergedNodeCount++;
        unmergedNodeCount -= 2;

        merged.emplace_back(false);
        merged[best_pair.from] = true;
        merged[best_pair.to] = true;
        // cout<<"merging "<<best_pair.from<<" "<<best_pair.to<<endl;

        NngNodes.emplace_back(mergedNngNode);

        currentMinimumPairs.clear();
    }

    int tempsize = 0;
    root = treeNodes.back(); //! the last node in treeNodes should be the root node.
    std::function<void(TreeNode *)> preOrderTraversal = [&](TreeNode *curNode)
    {
        if (curNode != nullptr)
        {
            int curId = curNode->id;
            preOrderTraversal(curNode->leftChild);
            preOrderTraversal(curNode->rightChild);
            tempsize++;
            // cout << "preing " << curId << endl;
        }
    };
    preOrderTraversal(root);
    nodeCount = tempsize;
}

double TreeTopology::nearestNeighborPairCost(NngNode a, NngNode b)
{
    return L1Dist(a, b); // simplist cost function, new function to be done in the future
}

double TreeTopology::nearestNeighborPairCost(TreeNode *a, TreeNode *b)
{
    return minManhattanDist(a, b) + 0.1 * (a->loadCapacitance + b->loadCapacitance);
}

void TreeTopology::initTreeNodes()
{
    int index = 0;
    assert(globalTreeNodes.size() == 0);
    for (Sink curSink : db->dbSinks)
    {
        assert(index == curSink.id);
        TreeNode *curRoot = new TreeNode(curSink.id);
        curRoot->loadCapacitance = curSink.capacitance;
        curRoot->trr = TRR(Segment(curSink), 0.0); // initilize TRR here!! so we can merge tree nodes
        curRoot->delay = 0.0;                      // initialize delay here!!
        globalTreeNodes.emplace_back(curRoot);
        index++;
    }
}

void TreeTopology::buildTreeUsingNearestNeighborGraph()
{
    //////////////////////////
    // Step 1: Calculate grid size
    //////////////////////////
    initTreeNodes();

    globalMerged.resize(globalTreeNodes.size(), false);
    // During a single CTS run, the boudingBox(of all tree nodes) might shrink but will not expand

    int leafCount = db->dbSinks.size();
    cout << "leafCount: " << leafCount << endl;
    // treeNodes is the S, after every merge merged, the merge node is added to S
    // vector<NngNodePair> currentMinimumPairs;

    int unmergedNodeCount = globalTreeNodes.size();

    // Step 1: calculated nearest neighbor graph
    while (unmergedNodeCount > 1) //! this while loop surely is the bottleneck for nng calculation. A potential "contribution"?
    {
        calculateNearestNeighbor(); // update the globalMinimumPairs
        //! choose pair with the smallest cost in currentMinimumPairs and merge
        while (!globalMinimumPairs.empty())
        {
            NngNodePair best_pair = globalMinimumPairs.top(); // top() since we are using a min_heap
            globalMinimumPairs.pop();
            if (globalMerged[best_pair.from] || globalMerged[best_pair.to])
            {
                continue;
            }

            TreeNode *mergeNode = new TreeNode(globalTreeNodes.size()); //!

            // TRR merge, remember to update delay and capacitane
            // capacitance of treeNodes are updated in DME

            mergeNode->leftChild = globalTreeNodes[best_pair.from];
            mergeNode->rightChild = globalTreeNodes[best_pair.to];

            TRRBasedMerge(mergeNode, mergeNode->leftChild, mergeNode->rightChild);

            globalTreeNodes[best_pair.from]->parent = mergeNode;
            globalTreeNodes[best_pair.to]->parent = mergeNode;

            globalTreeNodes.emplace_back(mergeNode);

            globalMerged.emplace_back(false);
            globalMerged[best_pair.from] = true;
            globalMerged[best_pair.to] = true;
            // cout<<"merging "<<best_pair.from<<" "<<best_pair.to<<endl;

            unmergedNodeCount--;

            if (unmergedNodeCount > 1)//avoid bug when we reach the root
            {
                //! push the nearest neighbor of the mergedNode to the priority queue
                double cost = numeric_limits<double>::max();
                int to = -1;
                for (int i = 0; i < globalTreeNodes.size(); i++)
                {
                    assert(globalTreeNodes[i]->id == i);
                    if (!globalMerged[i])
                    {

                        if ((i != mergeNode->id))
                        {
                            double newCost = nearestNeighborPairCost(globalTreeNodes[i], mergeNode);
                            if (newCost < cost)
                            {
                                cost = newCost;
                                to = i;
                            }
                        }
                        assert(to != -1);
                    }
                }
                assert(globalTreeNodes[to]->id == to);
                globalMinimumPairs.push(NngNodePair(mergeNode->id, globalTreeNodes[to]->id, cost));
            }
        }
    }

    int tempsize = 0;
    root = globalTreeNodes.back(); //! the last node in treeNodes should be the root node.
    std::function<void(TreeNode *)> preOrderTraversal = [&](TreeNode *curNode)
    {
        if (curNode != nullptr)
        {
            int curId = curNode->id;
            preOrderTraversal(curNode->leftChild);
            preOrderTraversal(curNode->rightChild);
            tempsize++;
            //cout << "preing " << curId << endl;
        }
    };
    preOrderTraversal(root);
    assert(tempsize==globalTreeNodes.size());
    nodeCount = globalTreeNodes.size();
}

void TreeTopology::initGrids()
{
    Rect boundingBox;

    int nodeCount = globalTreeNodes.size();

    double DOUBLE_MAX = __DBL_MAX__;

    double xMin = DOUBLE_MAX;
    double xMax = -DOUBLE_MAX;
    double yMin = DOUBLE_MAX;
    double yMax = -DOUBLE_MAX;

    for (TreeNode *curNode : globalTreeNodes)
    {
        double curNodeX = curNode->trr.getMiddlePoint().x;
        double curNodeY = curNode->trr.getMiddlePoint().y;
        xMin = min(xMin, curNodeX);
        xMax = max(xMax, curNodeX);
        yMin = min(xMin, curNodeY);
        yMax = max(xMax, curNodeY);
    }

    boundingBox.ll.x = xMin - EPS;
    boundingBox.ll.y = yMin - EPS;
    boundingBox.ur.x = xMax + EPS;
    boundingBox.ur.y = yMax + EPS;

    gridCountX = gridCountY = ceil(sqrt(nodeCount) * 0.5); // average 4 nodes per grid for n*n(gridCountX=gridCountY=n) grids, so nodeCount/(n*n)=4, n=0.5*sqrt(nodeCount)
    // ceil: at least 1 grid
    double gridWidth = boundingBox.getWidth() / gridCountX;
    double gridHeight = boundingBox.getHeight() / gridCountY;

    //! grid index:
    //! # 5
    //! # 4
    //! # 3
    //! # 2
    //! # 1
    //! # 0
    //! #  0 1 2 3 4 5
    grids.resize(gridCountX);
    for (int i = 0; i < gridCountX; i++)
    {
        grids[i].resize(gridCountY);
        for (int j = 0; j < gridCountY; j++)
        {
            grids[i][j] = NNGGrid();
            // cout<<"adding grid "<<i<<","<<j<<endl;
            //!!! +boundingBox.ll.x to get coordinates!!
            grids[i][j].ll.x = i * gridWidth + boundingBox.ll.x;
            grids[i][j].ll.y = j * gridHeight + boundingBox.ll.y;

            grids[i][j].width = gridWidth;
            grids[i][j].height = gridHeight;

            grids[i][j].ur.x = grids[i][j].ll.x + grids[i][j].width;
            grids[i][j].ur.y = grids[i][j].ll.y + grids[i][j].height;

            grids[i][j].area = gridWidth * gridHeight;

            grids[i][j].center.x = grids[i][j].ll.x + (float)0.5 * grids[i][j].width;
            grids[i][j].center.y = grids[i][j].ll.y + (float)0.5 * grids[i][j].height;
        }
    }

    for (TreeNode *curNode : globalTreeNodes)
    {
        double curNodeX = curNode->trr.getMiddlePoint().x;
        double curNodeY = curNode->trr.getMiddlePoint().y;
        int gridIndexX = (curNodeX - boundingBox.ll.x) / gridWidth;
        int gridIndexY = (curNodeY - boundingBox.ll.y) / gridHeight;

        assert(gridIndexX >= 0 && gridIndexX < gridCountX);
        assert(gridIndexY >= 0 && gridIndexY < gridCountY);

        grids[gridIndexX][gridIndexY].gridTreeNodes.push_back(curNode);
    }
}

void TreeTopology::calculateNearestNeighbor()
{
    for (int i = 0; i < globalTreeNodes.size(); i++)
    {
        assert(globalTreeNodes[i]->id == i);
        if (!globalMerged[i])
        {
            double cost = numeric_limits<double>::max();
            int to = -1;
            for (int j = 0; j < globalTreeNodes.size(); j++)
            {
                if ((i != j) && !globalMerged[j])
                {
                    double newCost = nearestNeighborPairCost(globalTreeNodes[i], globalTreeNodes[j]);
                    if (newCost < cost)
                    {
                        cost = newCost;
                        to = j;
                    }
                }
            }
            assert(to != -1);
            globalMinimumPairs.push(NngNodePair(globalTreeNodes[i]->id, globalTreeNodes[to]->id, cost)); //! treeNodes[i]->id==i, treeNodes[to]->id==to
        }
    }
}

void TreeTopology::updateVforOneGrid(NNGGrid &husband, vector<TreeNode *> wifes)
{
    // for (TreeNode *husbandSubtree : husband.gridTreeNodes)
    // {
    //     if (!globalMerged[husbandSubtree->id])
    //     {
    //         double cost = numeric_limits<double>::max();
    //         int to = -1;
    //         for (TreeNode *wifeSubtree : all tree nodes in wifes)
    //         {
    //             if (!globalMerged[wifeSubtree->id] && husbandSubtree->id != wifeSubtree->id)
    //             {
    //                 double newCost = nearestNeighborPairCost(husbandSubtree, wifeSubtree);
    //                 if (newCost < cost)
    //                 {
    //                     cost = newCost;
    //                     to = wifeSubtree->id;
    //                 }
    //             }
    //         }
    //         husband.V.push(NngNodePair(husbandSubtree->id, to, cost)); // V of grid k stores the min cost pair for every subtree in grid k
    //     }
    // }
}
