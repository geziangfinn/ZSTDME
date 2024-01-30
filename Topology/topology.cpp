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
                        if (nearestNeighborPaitCost(NngNodes[i], NngNodes[j]) < cost)
                        {
                            cost = nearestNeighborPaitCost(NngNodes[i], NngNodes[j]);
                            to = j;
                        }
                    }
                }
                assert(to != -1);
                currentMinimumPairs.emplace_back(NngNodePair(treeNodes[i]->id, treeNodes[to]->id, cost));
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

        treeNodes.emplace_back(mergeNode);
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
            cout << "preing " << curId << endl;
        }
    };
    preOrderTraversal(root);
    nodeCount = tempsize;
}

double TreeTopology::nearestNeighborPaitCost(NngNode a, NngNode b)
{
    return L1Dist(a, b); // simplist cost function, new function to be done in the future
}
