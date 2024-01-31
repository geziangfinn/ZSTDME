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
                
                ret.id = 1;// could be a single point intersection, which is an extreme case
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
    if (insideTRR(seg.lowerPoint) && insideTRR(seg.higherPoint))// ! entire segment is inside TRR, need snaking!
    {
        Segment ret = seg;
        ret.id = 0;
        return ret;
    }

    Segment ret;
    ret.id = -1;
    return ret;
}
