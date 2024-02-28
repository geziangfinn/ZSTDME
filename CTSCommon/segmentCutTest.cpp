#include "global.h"
#include "objects.h"
#include "plotter.h"
int main(int argc, char *argv[])
{

    double RAND_MAX_INVERSE = (float)1.0 / RAND_MAX;

    srand(time(0));

    double slope = rand();
    slope = slope * double(RAND_MAX_INVERSE);
    cout << "slope: " << slope << endl;
    if (double_greater(slope, 0.5))
    {
        slope = 1;
    }
    else
    {
        slope = -1;
    }

    Blockage block;
    Segment seg;
    int llx = rand();
    int lly = rand();
    int urx = rand();
    int ury = rand();

    llx = llx * RAND_MAX_INVERSE * 4000;
    lly = lly * RAND_MAX_INVERSE * 4000;
    urx = urx * RAND_MAX_INVERSE * 4000;
    ury = ury * RAND_MAX_INVERSE * 4000;

    if (llx > urx)
    {
        swap(llx, urx);
    }

    if (lly > ury)
    {
        swap(lly, ury);
    }

    block.ll = Point_2D(llx, lly);
    block.ur = Point_2D(urx, ury);

    int lpx = rand();
    int lpy = rand();
    lpx = lpx * RAND_MAX_INVERSE * 4000;
    lpy = lpy * RAND_MAX_INVERSE * 4000;
    int length = rand();

    length = length * RAND_MAX_INVERSE * 4000;

    length = min(length, 4000 - lpx);
    int hpy;
    if (slope == 1)
    {
        length = min(length, 4000 - lpy);
        hpy = lpy + length;
    }
    else if (slope == -1)
    {
        length = min(length, lpy);
        hpy = lpy - length;
    }
    else
    {
        cerr << "slope error\n";
        exit(0);
    }

    int hpx = lpx + length;

    seg=Segment(Point_2D(lpx, lpy),Point_2D(hpx, hpy));

    if (!(strcmp(argv[1] + 1, "randomCase") == 0)) // -input, argv[1]=='-'
    {
        block.ll = Point_2D(2802, 726);
        block.ur = Point_2D(3145, 793);
        seg=Segment(Point_2D(1537, 1251),Point_2D(2788, 0));
        // seg.lowerPoint = Point_2D(1537, 1251);
        // seg.higherPoint = Point_2D(2788, 0);
    }

    cout << "Input seg: " << seg << endl;

    cout << "Veritical lines: " << block.ll.x << " " << block.ur.x << endl;
    cout << "Horizontal lines: " << block.ll.y << " " << block.ur.y << endl;

    vector<Segment> results = segmentCutByBlockage(seg, block);

    cout << "Results: \n";
    for (Segment curSegment : results)
    {

        cout << curSegment << endl;
    }

    string outFilePath = "segCutTest.plt";
    ofstream outfile(outFilePath.c_str(), ios::out);

    outfile << " " << endl;
    outfile << "set terminal png size 4000,4000" << endl;
    outfile << "set output "
            << "\""
            << "segCutTest"
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

    outfile << "plot[:][:]  '-' w l lt 3 lw 2, '-'  w l lt 7 lw 16" << endl
            << endl;

    outfile << "# Blockage" << endl;
    plotBoxPLT(outfile, block.ll.x, block.ll.y, block.ll.x, block.ur.y, block.ur.x, block.ur.y, block.ur.x, block.ll.y);
    outfile << "EOF" << endl;

    outfile << "# Segment" << endl;
    plotLinePLT(outfile, seg.lowerPoint.x, seg.lowerPoint.y, seg.higherPoint.x, seg.higherPoint.y);
    outfile << "EOF" << endl;

    // outfile << "pause -1 'Press any key to close.'" << endl;
    outfile.close();

    system(("gnuplot " + outFilePath).c_str());

    cout << BLUE << "[Router]" << RESET << " - Visualize the bottom_up graph in \'" << outFilePath << "\'.\n";
}
