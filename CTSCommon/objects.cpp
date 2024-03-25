#include "objects.h"
#include <objects.h>
#include "plotter.h"
double Segment::slope()
{
    if (isPoint())
    {
        return 0.0;
    }
    if (higherPoint.x == lowerPoint.x)
    {
        return 0.0;
    }
    double slope = 1.0 * (lowerPoint.y - higherPoint.y) / (lowerPoint.x - higherPoint.x);
    if (!(double_equal(slope, 1.0) || double_equal(slope, -1.0)))
    {
        cout << endl
             << slope << endl;
    }
    assert(double_equal(slope, 1.0) || double_equal(slope, -1.0));
    return slope;
}

// Segment Segment::intersect(Segment rhs)
// {
//     //! -1 for no intersection, 0 for segment intersection, 1 for point intersection
//     double cur_slope = slope();
//     double rhs_slope = rhs.slope();
//     if (rhs.isPoint() && this->isPoint())
//     {
//         cout << "points intersect: " << rhs << " " << *this << endl;
//         Segment ret = rhs;
//         ret.id = -1;
//         return ret;
//     }
//     assert(!(rhs.isPoint() && this->isPoint()));

//     if (rhs.isPoint() || this->isPoint()) //! if current segment is intersecting a single grid point, or current segment is a point!(happend in top down phase)
//     {
//         // cout<<"point\n";
//         if (this->isPoint())
//         {
//             //! set rhs as the point
//             Segment swap;
//             swap = *this;
//             *this = rhs;
//             rhs = swap; //! potential bug if overwrite *this??
//         }
//         Segment ret = rhs;
//         if (segmentOnSameLine(rhs, *this)) // check if 4 points same line
//         {
//             if (double_lessorequal(lowerPoint.y, rhs.lowerPoint.y) && double_lessorequal(rhs.lowerPoint.y, higherPoint.y)) //! check if rhs.lowerPoint is on the segment
//             {
//                 Segment ret = rhs;
//                 ret.id = 1; // return single point intersection
//                 return ret;
//             }
//         }
//         ret.id = -1;
//         return ret;
//     }
//     if (double_equal(cur_slope, rhs_slope))
//     { // equal slope
//         // cout<<" "<<(rhs.lowerPoint.y - lowerPoint.y) * (higherPoint.x - lowerPoint.x)<<" "<<(higherPoint.y - lowerPoint.y) * (rhs.lowerPoint.x - lowerPoint.x)<<endl;
//         if (segmentOnSameLine(rhs, *this)) // check if 4 points same line
//         {
//             assert(rhs.lowerPoint.y <= rhs.higherPoint.y && lowerPoint.y <= higherPoint.y);
//             Point_2D upper, lower;
//             if (rhs.higherPoint.y < higherPoint.y)
//             {
//                 upper = rhs.higherPoint;
//             }
//             else
//             {
//                 upper = higherPoint;
//             }
//             if (rhs.lowerPoint.y > lowerPoint.y)
//             {
//                 lower = rhs.lowerPoint;
//             }
//             else
//             {
//                 lower = lowerPoint;
//             }
//             if (upper.y < lower.y)
//             {
//                 Segment ret;
//                 ret.id = -1;
//                 return ret;
//                 // cout << "No overlap between two segs on the line" << endl;
//                 // exit(1);
//             }
//             // cout<<"LOWER: "<<lower<<" UPPER: "<<upper<<endl;
//             Segment ret(lower, upper);
//             if (lower == upper)
//             {

//                 ret.id = 1; // could be a single point intersection, which is an extreme case
//             }
//             else
//             {
//                 ret.id = 0;
//             }
//             return ret;
//         }
//         else
//         {
//             Segment ret;
//             ret.id = -1;
//             return ret;
//         }
//     }
//     else
//     {
//         // cout<<"slope1: "<<cur_slope<<" slope2: "<<rhs_slope<<endl;
//         // cout<<"SEG1: "<<*this<<"SEG2: "<<rhs<<endl;
//         // different slope for two segments, might be 1 ;point or 0
//         // cout<<"check if two lines cross each other\n";
//         double A1 = higherPoint.y - lowerPoint.y;
//         // double B1 = higherPoint.x - lowerPoint.x;
//         double B1 = lowerPoint.x - higherPoint.x;
//         double C1 = A1 * lowerPoint.x + B1 * lowerPoint.y;
//         double A2 = rhs.higherPoint.y - rhs.lowerPoint.y;
//         // double B2 = rhs.higherPoint.x - rhs.lowerPoint.x;
//         double B2 = rhs.lowerPoint.x - rhs.higherPoint.x;
//         double C2 = A2 * rhs.lowerPoint.x + B2 * rhs.lowerPoint.y;
//         double det = A1 * B2 - A2 * B1;
//         double x = (B2 * C1 - B1 * C2) / det;
//         double y = (A1 * C2 - A2 * C1) / det;

//         Segment ret;
//         if (double_lessorequal(lowerPoint.y, y) && double_lessorequal(y, higherPoint.y) && double_lessorequal(rhs.lowerPoint.y, y) && double_lessorequal(y, rhs.higherPoint.y))
//         {
//             ret.lowerPoint = Point_2D(x, y);
//             ret.higherPoint = Point_2D(x, y);
//             //  cout<<"LOW: "<<ret.lowerPoint<<" UPP: "<<ret.higherPoint<<endl;
//             ret.id = 1; // return single point intersection
//         }
//         else
//         {
//             // cout<<"intersection point: "<<Point_2D(x, y)<<endl;
//             ret.id = -1;
//         }
//         return ret;
//     }
//     //! if return with id=-1, no intersection
// }

void TRR::drawTRR(ofstream &stream)
{
    vector<Point_2D> trr1_boundary_grid;
    if (core.slope() > 0)
    {
        trr1_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y - radius);
        trr1_boundary_grid.emplace_back(core.higherPoint.x + radius, core.higherPoint.y);
        trr1_boundary_grid.emplace_back(core.higherPoint.x, core.higherPoint.y + radius);
        trr1_boundary_grid.emplace_back(core.lowerPoint.x - radius, core.lowerPoint.y); // clock-wise
    }
    else if (core.slope() < 0)
    {
        trr1_boundary_grid.emplace_back(core.lowerPoint.x + radius, core.lowerPoint.y);
        trr1_boundary_grid.emplace_back(core.higherPoint.x, core.higherPoint.y + radius);
        trr1_boundary_grid.emplace_back(core.higherPoint.x - radius, core.higherPoint.y);
        trr1_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y - radius); // clock-wise
    }
    else
    { // leaf node
        trr1_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y - radius);
        trr1_boundary_grid.emplace_back(core.lowerPoint.x + radius, core.lowerPoint.y);
        trr1_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y + radius);
        trr1_boundary_grid.emplace_back(core.lowerPoint.x - radius, core.lowerPoint.y); // clock-wise
    }

    plotBoxPLT(stream,
               trr1_boundary_grid[0].x, trr1_boundary_grid[0].y,
               trr1_boundary_grid[1].x, trr1_boundary_grid[1].y,
               trr1_boundary_grid[2].x, trr1_boundary_grid[2].y,
               trr1_boundary_grid[3].x, trr1_boundary_grid[3].y);
}

void TRR::drawCore(ofstream &stream)
{
    plotLinePLT(stream, core.lowerPoint.x,
                core.lowerPoint.y,
                core.higherPoint.x,
                core.higherPoint.y);
}

bool TRR::insideTRR(Point_2D point)
{
    if (double_equal(radius, 0.0)) // when radius of TRR==0
    {
        Segment pointSeg = Segment(point, point);
        Segment seg = intersect(core, pointSeg);
        return seg.id == 1; // point intersection
    }
    vector<Segment> trr1_Sides = getBoundarys();

    vector<double> negtive_slope_interceps; // use interceps to determine if a point is in a TRR, just like linear programming, but here all slopes of constraints are 1 or -1
    vector<double> positive_slope_interceps;
    for (Segment side : trr1_Sides)
    {
        if (side.slope() > 0)
        {
            positive_slope_interceps.emplace_back(side.lowerPoint.y - side.lowerPoint.x);
        }
        else if (side.slope() < 0)
        {
            negtive_slope_interceps.emplace_back(side.lowerPoint.y + side.lowerPoint.x);
        }
    }
    sort(positive_slope_interceps.begin(), positive_slope_interceps.end());
    sort(negtive_slope_interceps.begin(), negtive_slope_interceps.end());

    // return (point.x + point.y) >= negtive_slope_interceps[0] && (point.x + point.y) <= negtive_slope_interceps[1] && (point.y - point.x) >= positive_slope_interceps[0] && (point.y - point.x) <= positive_slope_interceps[1];
    return double_greaterorequal(point.x + point.y, negtive_slope_interceps[0]) && double_lessorequal(point.x + point.y, negtive_slope_interceps[1]) && double_greaterorequal(point.y - point.x, positive_slope_interceps[0]) && double_lessorequal(point.y - point.x, positive_slope_interceps[1]);
}

Point_2D TRR::getMiddlePoint()
{
    double x = 0.5 * (core.lowerPoint.x, core.higherPoint.x);
    double y = 0.5 * (core.lowerPoint.y, core.higherPoint.y);
    return Point_2D(x, y);
}

vector<Segment> TRR::getBoundarys()
{
    // get 4 boundaries of a TRR
    vector<Point_2D> trr_boundary_grid;
    vector<Segment> trr_Sides;
    // cout<<"seg slope: "<<seg.slope()<<" lowerPoint: "<<seg.lowerPoint<<" higherPoint: "<<seg.higherPoint<<endl;
    if (double_greater(core.slope(), 0.0))
    {
        trr_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y - radius);
        trr_boundary_grid.emplace_back(core.higherPoint.x + radius, core.higherPoint.y);
        trr_boundary_grid.emplace_back(core.higherPoint.x, core.higherPoint.y + radius);
        trr_boundary_grid.emplace_back(core.lowerPoint.x - radius, core.lowerPoint.y); // counter clock-wise
    }
    else if (double_less(core.slope(), 0.0))
    {
        trr_boundary_grid.emplace_back(core.lowerPoint.x + radius, core.lowerPoint.y);
        trr_boundary_grid.emplace_back(core.higherPoint.x, core.higherPoint.y + radius);
        trr_boundary_grid.emplace_back(core.higherPoint.x - radius, core.higherPoint.y);
        trr_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y - radius); // counter clock-wise
    }
    else
    { // core is leaf node
        trr_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y - radius);
        trr_boundary_grid.emplace_back(core.lowerPoint.x + radius, core.lowerPoint.y);
        trr_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y + radius);
        trr_boundary_grid.emplace_back(core.lowerPoint.x - radius, core.lowerPoint.y); // counter clock-wise
    }

    for (int i = 0; i < 3; i++)
    {
        trr_Sides.emplace_back(trr_boundary_grid[i], trr_boundary_grid[i + 1]);
    }
    trr_Sides.emplace_back(trr_boundary_grid[3], trr_boundary_grid[0]);
    return trr_Sides;
}

vector<Segment> segmentCutByBlockage(Segment seg, Blockage block) // 4 lines(2 vertical and 2 horizontal) -> 9 regions
{
    vector<Segment> resultedSegments;
    // First: cut the segment with 2 vertical lines(x=k), //!x1 is to the left of x2(x1<x2)
    double x1 = block.ll.x;
    double x2 = block.ur.x;

    vector<Segment> segmentCutByVerticalLines = segmentCutBy2ParallelLines(seg, x1, x2, true);
    // Then: cut the resulted segments with 2 horizontal lines(y=k),//! y1<y2, cut every resulted segment from the last stage!!!!
    double y1 = block.ll.y;
    double y2 = block.ur.y;
    for (Segment curSegment : segmentCutByVerticalLines)
    {
        cout << "Segments cut by vertical lines: " << curSegment << endl;
        vector<Segment> segmentCutByHorizontalLines = segmentCutBy2ParallelLines(curSegment, y1, y2, false);
        resultedSegments.insert(resultedSegments.end(), segmentCutByHorizontalLines.begin(), segmentCutByHorizontalLines.end());
    }
    return resultedSegments;
}

vector<Segment> segmentCutByLine(Segment seg, double line, bool verticalOrHorizontal)
{
    // todo: what if the seg is a single point?
    //! a very important assumption here: slope of the segment == -1 or 1!
    vector<Segment> resultedSegments; // should return two segments
    if (verticalOrHorizontal)         // cut by vertical line, x=k
    {
        double segXMax = max(seg.lowerPoint.x, seg.higherPoint.x);
        double segXMin = min(seg.lowerPoint.x, seg.higherPoint.x);
        if (double_less(segXMax, line) || double_greater(segXMin, line)) // no intersection
        {
            resultedSegments.push_back(seg);
            return resultedSegments; // vector size == 1 means segment is not cut by line
        }
        assert(double_lessorequal(segXMin, line) && double_lessorequal(line, segXMax));

        if (double_equal(seg.slope(), 1.0))
        {
            // y=x+b, b=y1-x1
            double b = seg.lowerPoint.y - seg.lowerPoint.x; // pick the lower point as (x1,y1)

            // y=x+b, x=line
            double crossPointY = line + b;
            // crossPointX=line
            Point_2D crossPoint(line, crossPointY);
            resultedSegments.push_back(Segment(seg.lowerPoint, crossPoint));
            resultedSegments.push_back(Segment(seg.higherPoint, crossPoint));
        }
        else if (double_equal(seg.slope(), -1.0))
        {
            // y=-x+b, b=y1+x1
            double b = seg.lowerPoint.y + seg.lowerPoint.x;

            // y=b-x, x=line
            double crossPointY = b - line;
            // crossPointX=line
            Point_2D crossPoint(line, crossPointY);
            resultedSegments.push_back(Segment(seg.lowerPoint, crossPoint));
            resultedSegments.push_back(Segment(seg.higherPoint, crossPoint));
        }
        else
        {
            cout << RED << "slope error(x=k)!" << RESET << endl;
            exit(0);
        }
    }
    else // cut by horizontal line, y=k
    {
        if (double_less(seg.higherPoint.y, line) || double_greater(seg.lowerPoint.y, line)) // no cross point
        {
            resultedSegments.push_back(seg);
            return resultedSegments; // vector size == 1 means segment is not cut by line
        }
        assert(double_lessorequal(seg.lowerPoint.y, line) && double_lessorequal(line, seg.higherPoint.y));

        if (double_equal(seg.slope(), 1.0))
        {
            // y=x+b, b=y1-x1
            double b = seg.lowerPoint.y - seg.lowerPoint.x;

            // x=y-b, y=line
            double crossPointX = line - b;
            // crossPointY=line
            Point_2D crossPoint(crossPointX, line);
            resultedSegments.push_back(Segment(seg.lowerPoint, crossPoint));
            resultedSegments.push_back(Segment(seg.higherPoint, crossPoint));
        }
        else if (double_equal(seg.slope(), -1.0))
        {
            // y=-x+b, b=y1+x1
            double b = seg.lowerPoint.y + seg.lowerPoint.x;

            // x=b-y, y=line
            double crossPointX = b - line;
            // crossPointY=line
            Point_2D crossPoint(crossPointX, line);
            resultedSegments.push_back(Segment(seg.lowerPoint, crossPoint));
            resultedSegments.push_back(Segment(seg.higherPoint, crossPoint));
        }
        else
        {
            cout << RED << "slope error(y=k)!" << RESET << endl;
            exit(0);
        }
    }
    return resultedSegments;
}

vector<Segment> segmentCutBy2ParallelLines(Segment seg, double line1, double line2, bool verticalOrHorizontal)
{
    vector<Segment> resultedSegments; // return at most 3 segments
    if (double_greater(line1, line2))
    {
        std::swap(line1, line2); // assume line1 to the left of line2, or below line2
    }

    assert(double_less(line1, line2));

    if (verticalOrHorizontal) // cut by vertical lines, x=k
    {
        double segXMax = std::max(seg.lowerPoint.x, seg.higherPoint.x);
        double segXMin = std::min(seg.lowerPoint.x, seg.higherPoint.x);
        if (double_less(segXMax, line1) || double_greater(segXMin, line2)) // seg not cut by either lines
        {
            resultedSegments.push_back(seg); // resultedSegments.size==1, return the seg if no intersection
        }
        else if (double_lessorequal(line1, segXMin) && double_lessorequal(segXMax, line2)) // seg between two lines(also, not cut by either lines)
        {
            resultedSegments.push_back(seg); // resultedSegments.size==1, return the seg if no intersection
        }
        else if (double_less(segXMin, line1) && double_less(line2, segXMax)) // seg is cut by both lines
        {
            if (double_equal(seg.slope(), 1.0))
            {
                // y=x+b, b=y1-x1
                // crossPointX=line1/line2
                double b = seg.lowerPoint.y - seg.lowerPoint.x; // pick the lower point as (x1,y1)

                // y=x+b, x=line
                double crossPoint1Y = line1 + b;
                double crossPoint2Y = line2 + b;

                Point_2D crossPoint1(line1, crossPoint1Y);
                Point_2D crossPoint2(line2, crossPoint2Y);

                resultedSegments.push_back(Segment(seg.lowerPoint, crossPoint1));
                resultedSegments.push_back(Segment(crossPoint1, crossPoint2));
                resultedSegments.push_back(Segment(crossPoint2, seg.higherPoint)); // resultedSegments.size==3
            }
            else if (double_equal(seg.slope(), -1.0))
            {
                // y=-x+b, b=y1+x1
                // crossPointX=line1/line2
                double b = seg.lowerPoint.x + seg.lowerPoint.y; // pick the lower point as (x1,y1)

                // y=b-x, x=line
                double crossPoint1Y = b - line1;
                double crossPoint2Y = b - line2;

                Point_2D crossPoint1(line1, crossPoint1Y);
                Point_2D crossPoint2(line2, crossPoint2Y);

                resultedSegments.push_back(Segment(seg.lowerPoint, crossPoint1));
                resultedSegments.push_back(Segment(crossPoint1, crossPoint2));
                resultedSegments.push_back(Segment(crossPoint2, seg.higherPoint)); // resultedSegments.size==3
            }
            else
            {
                cout << RED << "slope error(x=k)!" << RESET << endl;
                exit(0);
            }
        }
        else // seg is cut by either line1 or line2
        {
            vector<Segment> segmentCutByX1 = segmentCutByLine(seg, line1, true);
            vector<Segment> segmentCutByX2 = segmentCutByLine(seg, line2, true);
            // segmentCutByX.size==1 means not cut by that line

            assert(!((segmentCutByX1.size() > 1) && (segmentCutByX2.size() > 1)));

            if (segmentCutByX1.size() > 1)
            {
                resultedSegments.insert(resultedSegments.end(), segmentCutByX1.begin(), segmentCutByX1.end());
            }
            if (segmentCutByX2.size() > 1)
            {
                resultedSegments.insert(resultedSegments.end(), segmentCutByX2.begin(), segmentCutByX2.end());
            }
            assert(resultedSegments.size() == 2); // resultedSegments.size==2
        }
    }
    else // cut by horizontal lines, y=k
    {
        double segYMax = seg.higherPoint.y;
        double segYMin = seg.lowerPoint.y;
        assert(segYMin < segYMax);
        if (double_less(segYMax, line1) || double_greater(segYMin, line2)) // seg not cut by either lines
        {
            resultedSegments.push_back(seg); // resultedSegments.size==1, return the seg if no intersection
        }
        else if (double_lessorequal(line1, segYMin) && double_lessorequal(segYMax, line2)) // seg between two lines(also, not cut by either lines)
        {
            resultedSegments.push_back(seg); // resultedSegments.size==1, return the seg if no intersection
        }
        else if (double_less(segYMin, line1) && double_less(line2, segYMax)) // seg is cut by both lines
        {
            if (double_equal(seg.slope(), 1.0))
            {
                // y=x+b, b=y1-x1
                // crossPointY=line1 or line2
                double b = seg.lowerPoint.y - seg.lowerPoint.x; // pick the lower point as (x1,y1)

                // x=y-b, y=line
                double crossPoint1X = line1 - b;
                double crossPoint2X = line2 - b;

                Point_2D crossPoint1(crossPoint1X, line1);
                Point_2D crossPoint2(crossPoint2X, line2);

                resultedSegments.push_back(Segment(seg.lowerPoint, crossPoint1));
                resultedSegments.push_back(Segment(crossPoint1, crossPoint2));
                resultedSegments.push_back(Segment(crossPoint2, seg.higherPoint)); // resultedSegments.size==3
            }
            else if (double_equal(seg.slope(), -1.0))
            {
                // y=-x+b, b=y1+x1
                // crossPointY=line1 or line2
                double b = seg.lowerPoint.x + seg.lowerPoint.y; // pick the lower point as (x1,y1)

                // x=b-y, y=line
                double crossPoint1X = b - line1;
                double crossPoint2X = b - line2;

                Point_2D crossPoint1(crossPoint1X, line1);
                Point_2D crossPoint2(crossPoint2X, line2);

                resultedSegments.push_back(Segment(seg.lowerPoint, crossPoint1));
                resultedSegments.push_back(Segment(crossPoint1, crossPoint2));
                resultedSegments.push_back(Segment(crossPoint2, seg.higherPoint)); // resultedSegments.size==3
            }
            else
            {
                cout << RED << "slope error(y=k)!" << RESET << endl;
                exit(0);
            }
        }
        else // seg is cut by either line1 or line2
        {
            vector<Segment> segmentCutByY1 = segmentCutByLine(seg, line1, false);
            vector<Segment> segmentCutByY2 = segmentCutByLine(seg, line2, false);
            // segmentCutByX.size==1 means not cut by that line

            assert(!((segmentCutByY1.size() > 1) && (segmentCutByY2.size() > 1)));

            if (segmentCutByY1.size() > 1)
            {
                resultedSegments.insert(resultedSegments.end(), segmentCutByY1.begin(), segmentCutByY1.end());
            }
            if (segmentCutByY2.size() > 1)
            {
                resultedSegments.insert(resultedSegments.end(), segmentCutByY2.begin(), segmentCutByY2.end());
            }
            assert(resultedSegments.size() == 2); // resultedSegments.size == 2
        }
    }
    return resultedSegments;
}

int regionNumber(Segment curSeg, Blockage curBlockage)
{
    double x1 = curBlockage.ll.x;
    double x2 = curBlockage.ur.x;
    double y1 = curBlockage.ll.y;
    double y2 = curBlockage.ur.y;

    int horizontalPositionNumber;
    int verticalPositionNumber;

    double midX = (curSeg.lowerPoint.x + curSeg.higherPoint.x) * 0.5;
    double midY = (curSeg.lowerPoint.y + curSeg.higherPoint.y) * 0.5;

    //! we use mid point here to determine the region of a seg,
    //! because although after segmentCutByBlockage,
    //! the entire resulted segment should be included in exactly ONE region,
    //! and any point on the segment should be ok, using higher or lower point may
    //! cause bugs because of boundary conditions. SO, we use mid point here.
    //! mid point is compatible even when the curSeg is a point, if we make sure there is no float accuracy problems

    if (double_lessorequal(midX, x1)) //? if seg is a point and is on the edge of regions,
                                      //? we should not determine it as a point in the MID region?
                                      //? because the connectivity of the MID region is bad? see skl paper
    {
        horizontalPositionNumber = LEFT;
    }
    else if (double_less(x1, midX) && double_less(midX, x2))
    {
        horizontalPositionNumber = MID_HORIZONTAL;
    }
    else if (double_lessorequal(x2, midX))
    {
        horizontalPositionNumber = RIGHT;
    }
    else
    {
        cerr << "horizontal region number error\n";
        exit(0);
    }

    if (double_lessorequal(midY, y1))
    {
        verticalPositionNumber = BOTTOM;
    }
    else if (double_less(y1, midY) && double_less(midY, y2))
    {
        verticalPositionNumber = MID_VERTICAL;
    }
    else if (double_lessorequal(y2, midY))
    {
        verticalPositionNumber = UP;
    }
    else
    {
        cerr << "vertical region number error\n";
        exit(0);
    }

    switch (verticalPositionNumber)
    {
    case UP:
        switch (horizontalPositionNumber)
        {
        case LEFT:
            return UP_LEFT;
            break;

        case MID_HORIZONTAL:
            return UP_MID;
            break;

        case RIGHT:
            return UP_RIGHT;
            break;

        default:
            cout << "error: horizontalPositionNumber is not in enum\n";
            exit(0);
            break;
        }
        break;

    case MID_VERTICAL:
        switch (horizontalPositionNumber)
        {
        case LEFT:
            return MID_LEFT;
            break;

        case MID_HORIZONTAL:
            return MID_MID;
            break;

        case RIGHT:
            return MID_RIGHT;
            break;

        default:
            cout << "error: horizontalPositionNumber is not in enum\n";
            exit(0);
            break;
        }
        break;

    case BOTTOM:
        switch (horizontalPositionNumber)
        {
        case LEFT:
            return BOTTOM_LEFT;
            break;

        case MID_HORIZONTAL:
            return BOTTOM_MID;
            break;

        case RIGHT:
            return BOTTOM_RIGHT;
            break;

        default:
            cout << "error: horizontalPositionNumber is not in enum\n";
            exit(0);
            break;
        }
        break;

    default:
        cout << "error: vertcalPositionNumber is not in enum\n";
        exit(0);
        break;
    }
}

bool insideRect(Point_2D point, Rect rectangle)
{
    Interval horizontal(rectangle.ll.x, rectangle.ur.x);
    Interval vertical(rectangle.ll.y, rectangle.ur.y);
    return horizontal.inside(point.x) && vertical.inside(point.y);
}

Point_2D moveToClosestBoundary(Point_2D point, Rect rectangle)
{
    if (!insideRect(point, rectangle))
    {
        return point;
    }
    double toTop = rectangle.ur.y - point.y;
    double toBottom = point.y - rectangle.ll.y;
    double toLeft = point.x - rectangle.ll.x;
    double toRight = rectangle.ur.x - point.x;
    vector<double> distances;
    distances.push_back(toBottom);
    distances.push_back(toRight);
    distances.push_back(toLeft);
    distances.push_back(toTop);

    sort(distances.begin(), distances.end());

    if (double_equal(distances[0], toTop))
    {
        point.y = rectangle.ur.y;
        return point;
    }

    if (double_equal(distances[0], toBottom))
    {
        point.y = rectangle.ll.y;
        return point;
    }

    if (double_equal(distances[0], toLeft))
    {
        point.x = rectangle.ll.x;
        return point;
    }

    if (double_equal(distances[0], toRight))
    {
        point.x = rectangle.ur.x;
        return point;
    }

    return point;
}

void updateMergeCapacitance(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb)
{

    double mergeCapacitance = nodeLeft->loadCapacitance + nodeRight->loadCapacitance + UNIT_CAPACITANCE * (ea + eb);
    // 考虑了buffer insertion的电容update
    //  if(delta_C > c_constraint){
    //      nodeMerge->load_capacitance = 300;
    //      nodeMerge->buffered=true;
    //      buffercount++;
    //      //nodeMerge->needBuffer = 1;
    //      return;
    //  }
    nodeMerge->loadCapacitance = mergeCapacitance;
}

void updateMergeDelay(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb)
{
    double delayToLeft;
    double delayToRight;

    int delayModel = ELMORE_DELAY;
    if (gArg.CheckExist("linearDelay"))
    {
        delayModel = LINEAR_DELAY;
    }

    if (delayModel == LINEAR_DELAY)
    {
        delayToLeft = nodeLeft->delay + ea;
        delayToRight = nodeRight->delay + eb;
    }
    else if (delayModel == ELMORE_DELAY) // refer to the abk paper for this
    {
        delayToLeft = nodeLeft->delay + 0.5 * UNIT_RESISTANCE * UNIT_CAPACITANCE * ea * ea +
                      UNIT_RESISTANCE * nodeLeft->loadCapacitance * ea;
        delayToRight = nodeRight->delay + 0.5 * UNIT_RESISTANCE * UNIT_CAPACITANCE * eb * eb +
                       UNIT_RESISTANCE * nodeRight->loadCapacitance * eb;
    }

    nodeMerge->delay = max(delayToLeft, delayToRight);
}

double solveForX(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, double L) // refer to the abk paper for this
{
    // x is the x in abk paper, and ea=x, eb=L-x,
    double numerator = (nodeRight->delay - nodeLeft->delay) + UNIT_RESISTANCE * L * (nodeRight->loadCapacitance + 0.5 * UNIT_CAPACITANCE * L);
    double denominator = UNIT_RESISTANCE * (nodeLeft->loadCapacitance + nodeRight->loadCapacitance + UNIT_CAPACITANCE * L);
    double x = numerator / denominator;
    if (!double_less(x, 0.0) && !double_greater(x, 0.0)) //!!!!
    {
        x = 0.0;
    }
    return x;
}

double solveForLPrime(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, int tag)
{

    // tag = 0: |eb| = L'
    // tag = 1: |ea| = L'
    double alphaC;
    double numerator;
    if (tag == 0)
    {
        alphaC = UNIT_RESISTANCE * nodeRight->loadCapacitance; // see abk paper
        numerator = sqrt(2 * UNIT_RESISTANCE * UNIT_CAPACITANCE * (nodeLeft->delay - nodeRight->delay) + alphaC * alphaC) - alphaC;
    }
    else
    {
        alphaC = UNIT_RESISTANCE * nodeLeft->loadCapacitance;
        numerator = sqrt(2 * UNIT_RESISTANCE * UNIT_CAPACITANCE * (nodeRight->delay - nodeLeft->delay) + alphaC * alphaC) - alphaC;
    }
    return numerator / (UNIT_RESISTANCE * UNIT_CAPACITANCE);
}

void drawTRRPair(string name, TRR trr1, TRR trr2)
{

    string plotPath;
    string benchmarkName;
    if (!gArg.GetString("plotPath", &plotPath))
    {
        plotPath = "./";
    }
    gArg.GetString("benchmarkName", &benchmarkName);

    string outFilePath = plotPath + benchmarkName + name + "_TRR_PAIR.plt";
    ofstream outfile(outFilePath.c_str(), ios::out);

    cout << "outputFilPath: " << endl;

    outfile << " " << endl;
    outfile << "set terminal png size 4000,4000" << endl;
    outfile << "set output "
            << "\"" << plotPath + benchmarkName + name << "_TRR_PAIR"
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

    outfile << "plot[:][:]  '-' w l lt 3 lw 2, '-'  w l lt 7 lw 10" << endl
            << endl;

    outfile << "# TRR" << endl;
    trr1.drawTRR(outfile);
    trr2.drawTRR(outfile);
    outfile << "EOF" << endl;

    outfile << "# TRR cores" << endl;
    trr1.drawCore(outfile);
    trr2.drawCore(outfile);
    outfile << "EOF" << endl;

    // outfile << "pause -1 'Press any key to close.'" << endl;
    outfile.close();

    system(("gnuplot " + outFilePath).c_str());
    cout << BLUE << "[Router]" << RESET << " - Visualize the TRR pair in \'" << outFilePath << "\'.\n";
}

Segment TRRintersectTRR(TRR trr1, TRR trr2)
{
    // get four edges
    // cout << "Merging: " << trr1 << " and " << trr2 << endl;
    vector<Segment> trr1_Sides = trr1.getBoundarys();
    vector<Segment> trr2_Sides = trr2.getBoundarys();
    if (double_equal(trr1.radius, 0.0) && double_equal(trr2.radius, 0.0))
    {
        if (trr1.core == trr2.core)
        {
            Segment ret = trr1.core;
            ret.id = 0;
            return ret;
        }
    }
    assert(!(double_equal(trr1.radius, 0.0) && double_equal(trr2.radius, 0.0)));

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // todo: what if a core is a leaf and its radius is 0? judge if its in a trr, but we didn't run into this case?
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //! if there is one trr's radius = 0
    if (double_equal(trr1.radius, 0.0) || double_equal(trr2.radius, 0.0))
    {
        if (double_equal(trr1.radius, 0.0))
        {
            Segment ret = TRRintersectSeg(trr2, trr1.core);
            if (ret.id != -1)
            {
                return ret;
            }
            cout << endl
                 << trr2.insideTRR(trr1.core.lowerPoint) << " ff " << trr2.insideTRR(trr1.core.higherPoint) << endl;
        }
        else
        {
            Segment ret = TRRintersectSeg(trr1, trr2.core);
            if (ret.id != -1)
            {
                return ret;
            }
            cout << endl
                 << trr1.insideTRR(trr2.core.lowerPoint) << " gg " << trr1.insideTRR(trr2.core.higherPoint) << endl;
        }

        cout << "Cannot find intersection between two TRRs when one TRR has radius == 0" << endl;
        for (auto &seg1 : trr1_Sides)
        {
            for (auto &seg2 : trr2_Sides)
            {
                if (double_equal(seg1.slope(), seg2.slope()))
                {
                    cout << "equal slope: " << endl;
                    // check if 4 point same line but rejected due to precision problems
                    cout << (seg2.lowerPoint.y - seg1.lowerPoint.y) * (seg1.higherPoint.x - seg1.lowerPoint.x) << " " << (seg1.higherPoint.y - seg1.lowerPoint.y) * (seg2.lowerPoint.x - seg1.lowerPoint.x);
                }
            }
        }
        drawTRRPair("bottomup_debug", trr1, trr2);
        Segment ret;
        ret.id = -1;
        return ret;
    }
    //!  if both trr's radius > 0
    // cout << "Print trr1's sides" << endl;
    // for (auto& seg1 : trr1_Sides) {
    //     cout << seg1 << endl;
    // }

    // cout << "Print trr2's sides" << endl;

    // for (auto& seg2 : trr2_Sides) {
    //     cout << seg2 << endl;
    // }

    // for 4*4 check intersect
    Segment tempseg; // we need this to prevent one scenario:
    // 当两个TRR有边重合（斜率相等），令斜率相等的两边为TRR1.x和TRR2.x，那么，对于TRR1，其中当然还有边TRR1.y与TRR1.x垂直。那么这个循环有可能先返回TRR1.y与TRR2.x的交点！导致错误
    // 为此，当获得point intersection时，不急着返回，再等等看有没有segment
    // intersection
    tempseg.id = -1;
    for (auto &seg1 : trr1_Sides)
    {
        for (auto &seg2 : trr2_Sides)
        {
            Segment seg = intersect(seg1, seg2);
            if (seg.id == 0)
            {
                return seg;
            }
            if (seg.id == 1)
            {
                tempseg = seg;
            }
        }
    }
    if (tempseg.id == 1)
    {
        return tempseg;
    }

    cout << "Cannot find intersection between two TRRs" << endl;

    for (auto &seg1 : trr1_Sides)
    {
        for (auto &seg2 : trr2_Sides)
        {
            if (double_equal(seg1.slope(), seg2.slope()))
            {
                cout << "equal slope: " << endl;
                // check if 4 point same line but rejected due to precision problems
                cout << (seg2.lowerPoint.y - seg1.lowerPoint.y) * (seg1.higherPoint.x - seg1.lowerPoint.x) << " " << (seg1.higherPoint.y - seg1.lowerPoint.y) * (seg2.lowerPoint.x - seg1.lowerPoint.x) << endl;
            }
        }
    }
    drawTRRPair("bottomup_debug", trr1, trr2);
    Segment ret;
    ret.id = -1;
    return ret;
}

void TRRBasedMerge(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight)
{
    double L = minManhattanDist(nodeLeft, nodeRight);
    double e_a; // radius of TRR_a
    double e_b; // radius of TRR_b, e_a, e_b are used in the book or the abk paper as well
                //! e_a for leftChild and e_b for rightChild
    // using Elmore delay here
    double x = solveForX(nodeLeft, nodeRight, nodeMerge, L);
    if (double_greaterorequal(x, 0.0) && double_lessorequal(x, L))
    {
        e_a = x;
        e_b = L - x;
    }
    else if (double_less(x, 0.0))
    {
        e_a = 0.0;
        e_b = solveForLPrime(nodeLeft, nodeRight, nodeMerge, 0);
        assert(double_greater(e_b, L));
    }
    else if (double_less(L, x))
    {
        e_b = 0.0;
        e_a = solveForLPrime(nodeLeft, nodeRight, nodeMerge, 1);
        assert(double_greater(e_a, L));
    }

    nodeLeft->trr.radius = e_a; //! e_a for leftChild and e_b for rightChild
    nodeRight->trr.radius = e_b;

    // intersect trr_a, trr_b to get ms_v
    Segment ms_v = TRRintersectTRR(nodeLeft->trr, nodeRight->trr);

    // todo: nine region method to solve for feasible merging segment, and recalculate ea and eb

    if (ms_v.id == -1)
    {
        cout << "Merge failure" << endl;
        cout << "TRR 1: " << nodeLeft->trr << endl;
        cout << "TRR 2: " << nodeRight->trr << endl;

        exit(1);
    }
    nodeMerge->trr.core = ms_v;

    updateMergeDelay(nodeMerge, nodeLeft, nodeRight, e_a, e_b);
    updateMergeCapacitance(nodeMerge, nodeLeft, nodeRight, e_a, e_b); //? there is a same function in RLC_calculation
}

bool segmentOnSameLine(Segment seg1, Segment seg2) //
{
    //! assumptions of this function!!: this function is for a segment and a point or for 2 segments with same slopes
    //  use lower point of seg1 and seg2 to calculate the slope of connection
    //! bug: when a point is right on the lower or higher point of a segment
    if (seg1.lowerPoint == seg2.lowerPoint)
    {
        return true;
    }
    double slope = (seg1.lowerPoint.y - seg2.lowerPoint.y) / (seg1.lowerPoint.x - seg2.lowerPoint.x);
    // cout<<"delta slope: "<<slope<<endl;
    if (double_equal(seg1.slope(), 0.0))
    {
        return double_equal(slope, seg2.slope());
    }
    else
    {
        // cout<<setprecision(9)<<"1: "<<slope<<" 2: "<<seg1.slope()<<" equal: "<<double_equal(slope, seg1.slope())<<endl;
        return double_equal(slope, seg1.slope());
    }
    return false;
}

Segment TRRintersectSeg(TRR trr, Segment seg)
{
    //! -1 for no intersection!
    vector<Segment> trr_Sides = trr.getBoundarys();
    for (Segment side : trr_Sides)
    {
        Segment intersection = intersect(seg, side);
        if (intersection.id == 0)
        {
            return intersection;
        }
        else if (intersection.id == 1) // single point intersection
        {
            // cout << "debughere7.5\n";
            if (trr.insideTRR(seg.lowerPoint))
            {
                // cout << "debughere8\n";
                Segment ret(intersection.lowerPoint, seg.lowerPoint);
                ret.id = 0;
                return ret;
            }
            else if (trr.insideTRR(seg.higherPoint))
            {
                // cout << "debughere9\n";
                Segment ret(intersection.lowerPoint, seg.higherPoint);
                ret.id = 0;
                return ret;
            }
            else
            {
                Segment ret(intersection);
                ret.id = 1;
                return ret;
            }
        }
    }
    if (trr.insideTRR(seg.lowerPoint) && trr.insideTRR(seg.higherPoint)) // ! entire segment is inside TRR, need snaking!
    {
        Segment ret = seg;
        ret.id = 0;
        return ret;
    }

    Segment ret;
    ret.id = -1;
    return ret;
}

Segment intersect(Segment lhs, Segment rhs)
{
    //! -1 for no intersection, 0 for segment intersection, 1 for point intersection
    double lhs_slope = lhs.slope();
    double rhs_slope = rhs.slope();
    if (rhs.isPoint() && lhs.isPoint())
    {
        // cout << "points intersect: " << rhs << " " << lhs << endl;
        Segment ret = rhs;
        if (lhs == rhs)
        {
            ret = lhs;
            ret.id = 1;
            return ret;
        }

        ret.id = -1;
        return ret;
    }
    assert(!(rhs.isPoint() && lhs.isPoint()));

    if (rhs.isPoint() || lhs.isPoint()) //! if current segment is intersecting a single grid point, or current segment is a point!(happend in top down phase)
    {
        if (lhs.isPoint())
        {
            //! set rhs as the point
            Segment swap;
            swap = lhs;
            lhs = rhs;
            rhs = swap;
        }
        Segment ret = rhs;
        if (segmentOnSameLine(rhs, lhs)) // check if 4 points same line
        {
            if (double_lessorequal(lhs.lowerPoint.y, rhs.lowerPoint.y) && double_lessorequal(rhs.lowerPoint.y, lhs.higherPoint.y)) //! check if rhs.lowerPoint is on the segment
            {
                Segment ret = rhs;
                ret.id = 1; // return single point intersection
                return ret;
            }
        }
        ret.id = -1;
        return ret;
    }
    if (double_equal(lhs_slope, rhs_slope))
    { // equal slope
        // cout<<" "<<(rhs.lowerPoint.y - lowerPoint.y) * (higherPoint.x - lowerPoint.x)<<" "<<(higherPoint.y - lowerPoint.y) * (rhs.lowerPoint.x - lowerPoint.x)<<endl;
        if (segmentOnSameLine(rhs, lhs)) // check if 4 points same line
        {
            assert(rhs.lowerPoint.y <= rhs.higherPoint.y && lhs.lowerPoint.y <= lhs.higherPoint.y);
            Point_2D upper, lower;
            if (rhs.higherPoint.y < lhs.higherPoint.y)
            {
                upper = rhs.higherPoint;
            }
            else
            {
                upper = lhs.higherPoint;
            }
            if (rhs.lowerPoint.y > lhs.lowerPoint.y)
            {
                lower = rhs.lowerPoint;
            }
            else
            {
                lower = lhs.lowerPoint;
            }
            if (upper.y < lower.y)
            {
                Segment ret;
                ret.id = -1;
                return ret;
                // cout << "No overlap between two segs on the line" << endl;
                // exit(1);
            }
            // cout<<"LOWER: "<<lower<<" UPPER: "<<upper<<endl;
            Segment ret(lower, upper);
            if (lower == upper)
            {

                ret.id = 1; // could be a single point intersection, which is an extreme case
            }
            else
            {
                ret.id = 0;
            }
            return ret;
        }
        else
        {
            Segment ret;
            ret.id = -1;
            return ret;
        }
    }
    else
    {
        // cout<<"slope1: "<<cur_slope<<" slope2: "<<rhs_slope<<endl;
        // cout<<"SEG1: "<<*this<<"SEG2: "<<rhs<<endl;
        // different slope for two segments, might be 1 point or 0
        //! check if two lines cross each other
        double A1 = lhs.higherPoint.y - lhs.lowerPoint.y;
        // double B1 = higherPoint.x - lowerPoint.x;
        double B1 = lhs.lowerPoint.x - lhs.higherPoint.x;
        double C1 = A1 * lhs.lowerPoint.x + B1 * lhs.lowerPoint.y;
        double A2 = rhs.higherPoint.y - rhs.lowerPoint.y;
        // double B2 = rhs.higherPoint.x - rhs.lowerPoint.x;
        double B2 = rhs.lowerPoint.x - rhs.higherPoint.x;
        double C2 = A2 * rhs.lowerPoint.x + B2 * rhs.lowerPoint.y;
        double det = A1 * B2 - A2 * B1;
        double x = (B2 * C1 - B1 * C2) / det;
        double y = (A1 * C2 - A2 * C1) / det;

        Segment ret;
        if (double_lessorequal(lhs.lowerPoint.y, y) && double_lessorequal(y, lhs.higherPoint.y) && double_lessorequal(rhs.lowerPoint.y, y) && double_lessorequal(y, rhs.higherPoint.y))
        {
            ret.lowerPoint = Point_2D(x, y);
            ret.higherPoint = Point_2D(x, y);
            //  cout<<"LOW: "<<ret.lowerPoint<<" UPP: "<<ret.higherPoint<<endl;
            ret.id = 1; // return single point intersection
        }
        else
        {
            ret.id = -1;
        }
        return ret;
    }
    //! if return with id=-1, no intersection
}
