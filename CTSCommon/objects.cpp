#include <objects.h>
#include "plotter.h"
double Segment::slope()
{
    if (isPoint())
    {
        return 0;
    }
    if (higherPoint.x == lowerPoint.x)
    {
        return 0;
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

Segment Segment::intersect(Segment rhs)
{
    //! -1 for no intersection, 0 for segment intersection, 1 for point intersection
    double cur_slope = slope();
    double rhs_slope = rhs.slope();
    assert(!(rhs.isPoint() && this->isPoint()));

    if (rhs.isPoint() || this->isPoint()) //! if current segment is intersecting a single grid point, or current segment is a point!(happend in top down phase)
    {
        if (this->isPoint())
        {
            //! set rhs as the point
            Segment swap;
            swap = *this;
            *this = rhs;
            rhs = swap; //! potential bug if overwrite *this??
        }
        Segment ret = rhs;
        if (double_equal((rhs.lowerPoint.y - lowerPoint.y) * (higherPoint.x - lowerPoint.x), (higherPoint.y - lowerPoint.y) * (rhs.lowerPoint.x - lowerPoint.x))) // check if 4 points same line
        {
            if (double_lessorequal(lowerPoint.y, rhs.lowerPoint.y) && double_lessorequal(rhs.lowerPoint.y, higherPoint.y)) //! check if rhs.lowerPoint is on the segment
            {
                Segment ret = rhs;
                ret.id = 1; // return single point intersection
                return ret;
            }
        }
        ret.id = -1;
        return ret;
    }
    if (double_equal(cur_slope, rhs_slope))
    { // equal slope
        // cout<<" "<<(rhs.lowerPoint.y - lowerPoint.y) * (higherPoint.x - lowerPoint.x)<<" "<<(higherPoint.y - lowerPoint.y) * (rhs.lowerPoint.x - lowerPoint.x)<<endl;
        if (double_equal((rhs.lowerPoint.y - lowerPoint.y) * (higherPoint.x - lowerPoint.x), (higherPoint.y - lowerPoint.y) * (rhs.lowerPoint.x - lowerPoint.x))) // check if 4 points same line
        {
            assert(rhs.lowerPoint.y <= rhs.higherPoint.y && lowerPoint.y <= higherPoint.y);
            Point_2D upper, lower;
            if (rhs.higherPoint.y < higherPoint.y)
            {
                upper = rhs.higherPoint;
            }
            else
            {
                upper = higherPoint;
            }
            if (rhs.lowerPoint.y > lowerPoint.y)
            {
                lower = rhs.lowerPoint;
            }
            else
            {
                lower = lowerPoint;
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
        double A1 = higherPoint.y - lowerPoint.y;
        // double B1 = higherPoint.x - lowerPoint.x;
        double B1 = lowerPoint.x - higherPoint.x;
        double C1 = A1 * lowerPoint.x + B1 * lowerPoint.y;
        double A2 = rhs.higherPoint.y - rhs.lowerPoint.y;
        // double B2 = rhs.higherPoint.x - rhs.lowerPoint.x;
        double B2 = rhs.lowerPoint.x - rhs.higherPoint.x;
        double C2 = A2 * rhs.lowerPoint.x + B2 * rhs.lowerPoint.y;
        double det = A1 * B2 - A2 * B1;
        double x = (B2 * C1 - B1 * C2) / det;
        double y = (A1 * C2 - A2 * C1) / det;

        Segment ret;
        if (double_lessorequal(lowerPoint.y, y) && double_lessorequal(y, higherPoint.y) && double_lessorequal(rhs.lowerPoint.y, y) && double_lessorequal(y, rhs.higherPoint.y))
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
    vector<Point_2D> trr1_boundary_grid;
    vector<Segment> trr1_Sides;
    if (double_equal(radius, 0.0)) // when radius of TRR==0
    {
        Segment pointSeg = Segment(point, point);
        Segment seg = core.intersect(pointSeg);
        return seg.id == 1; // point intersection
    }

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
    { // core is leaf node
        trr1_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y - radius);
        trr1_boundary_grid.emplace_back(core.lowerPoint.x + radius, core.lowerPoint.y);
        trr1_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y + radius);
        trr1_boundary_grid.emplace_back(core.lowerPoint.x - radius, core.lowerPoint.y); // clock-wise
    }

    for (int i = 0; i < 3; i++)
    {
        trr1_Sides.emplace_back(trr1_boundary_grid[i], trr1_boundary_grid[i + 1]);
    }
    trr1_Sides.emplace_back(trr1_boundary_grid[3], trr1_boundary_grid[0]);

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

    return (point.x + point.y) >= negtive_slope_interceps[0] && (point.x + point.y) <= negtive_slope_interceps[1] && (point.y - point.x) >= positive_slope_interceps[0] && (point.y - point.x) <= positive_slope_interceps[1];
}

Segment TRR::TRRintersectSeg(Segment &seg)
{
    //! -1 for no intersection!
    vector<Point_2D> trr_boundary_grid;
    vector<Segment> trr_Sides;
    // cout<<"seg slope: "<<seg.slope()<<" lowerPoint: "<<seg.lowerPoint<<" higherPoint: "<<seg.higherPoint<<endl;
    trr_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y - radius);
    trr_boundary_grid.emplace_back(core.lowerPoint.x + radius, core.lowerPoint.y);
    trr_boundary_grid.emplace_back(core.lowerPoint.x, core.lowerPoint.y + radius);
    trr_boundary_grid.emplace_back(core.lowerPoint.x - radius, core.lowerPoint.y); // counter clock-wise
    for (int i = 0; i < 3; i++)
    {
        trr_Sides.emplace_back(trr_boundary_grid[i], trr_boundary_grid[i + 1]);
    }
    trr_Sides.emplace_back(trr_boundary_grid[3], trr_boundary_grid[0]);
    // for (auto& seg1 : trr_Sides) {
    //     cout << seg1 << endl;
    // }
    // cout<<"\ntop-dwon\n";
    for (auto &side : trr_Sides)
    {
        Segment intersection = side.intersect(seg);
        if (intersection.id != -1)
        {
            return intersection;
        }
    }
    if (insideTRR(seg.lowerPoint) && insideTRR(seg.higherPoint)) // ! entire segment is inside TRR, need snaking!
    {
        Segment ret = seg;
        ret.id = 0;
        return ret;
    }

    Segment ret;
    ret.id = -1;
    return ret;
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
        cout<<"Segments cut by vertical lines: "<<curSegment<<endl;
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
        assert(segYMin<segYMax);
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
