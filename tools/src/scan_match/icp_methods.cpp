#include <scan_match/icp_methods.hpp>

MyPointCloud2D Verwerfung::selectVewerfung(VerwerfungsMethode verwerfungs_methode, MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark, const State &state)
{
    MyPointCloud2D output;
    switch (verwerfungs_methode)
    {
    case VerwerfungsMethode::constantValue:
        output = constantValue(map_corrs, scans, map_carpark);
        break;
    case VerwerfungsMethode::constantAndRisingDistance:
        output = constantAndRisingDistance(map_corrs, scans, map_carpark, state);
        break;
    }
    return output;
}
void Gewichtung::selectGewichtung(GewichtungsMethode gewichtungs_methode, MyPointCloud2D &scans)
{
    switch (gewichtungs_methode)
    {
    case GewichtungsMethode::applyNothing:
        applyNothing(scans);
        break;
    case GewichtungsMethode::divideByAllCorrespondingDistances:
        divideByAllCorrespondingDistances(scans);
        break;
    case GewichtungsMethode::setZeroIfTwoCorrespondings:
        setZeroIfTwoCorrespondings(scans);
        break;
    }
}

float Verwerfung::getFiltDistance(float error_before_matching, float ratio_corres_last_timestep)
{
    float filt_distance;
    if (error_before_matching >= 3)
    {
        filt_distance = 5;
    }
    else if ((error_before_matching < 3) && (error_before_matching >= 2))
    {
        filt_distance = 3.8;
    }
    else if ((error_before_matching < 2) && (error_before_matching >= 1.5))
    {
        filt_distance = 2.7;
    }
    else if ((error_before_matching < 1.5) && (error_before_matching >= 1))
    {
        filt_distance = 1.9;
    }
    else
    {
        filt_distance = 1.0;
    }

    if (ratio_corres_last_timestep > 0.8)
    {
        filt_distance = 1.0;
    }
    filt_distance = error_before_matching + 0.2;
    return filt_distance;
}

MyPointCloud2D Verwerfung::constantValue(MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark)
{
    MyPointCloud2D temp_scans;
    MyPointCloud2D temp_map_corrs;
    MyPoint temp_pt;
    int index;
    //Removing correspondencies that are too far apart
    for (size_t i = 0; i < scans.distances.size(); i++)
    {
        if (scans.distances[i] < constant_value)
        {
            temp_scans.distances.push_back(scans.distances[i]);
            temp_scans.ids.push_back(i);
            temp_scans.weights.push_back(1);
            temp_pt.x = scans.pts[i].x;
            temp_pt.y = scans.pts[i].y;
            temp_scans.pts.push_back(temp_pt);

            temp_map_corrs.distances.push_back(map_corrs.distances[i]);
            temp_map_corrs.ids.push_back(i);
            temp_map_corrs.weights.push_back(1);
            index = scans.ids[i];
            temp_pt.x = map_carpark.pts[index].x;
            temp_pt.y = map_carpark.pts[index].y;
            temp_map_corrs.pts.push_back(temp_pt);
        }
    }
    map_corrs = temp_map_corrs;
    return temp_scans;
}

// @The filter distances in between will rise linearly: y = m * x + b ; m = (max-min)/100 , b = min
// @@Input: 1) Put in minimum filter distance 2) put in maximum filter distance at 100 meters
MyPointCloud2D Verwerfung::constantAndRisingDistance(MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark, const State &state)
{
    MyPointCloud2D temp_scans;
    MyPointCloud2D temp_map_corrs;
    float laser_car_distance;
    float filt_distance;
    MyPoint temp_pt;
    int index;
    for (size_t i = 0; i < scans.distances.size(); i++)
    {
        laser_car_distance = sqrt((scans.pts[i].x - state.x) * (scans.pts[i].x - state.x) + (scans.pts[i].y - state.y) * (scans.pts[i].y - state.y));
        filt_distance = ((max_distance - min_distance) / 100.0) * laser_car_distance + min_distance;
        if (scans.distances[i] < filt_distance)
        {
            temp_scans.distances.push_back(scans.distances[i]);
            temp_scans.ids.push_back(i);
            temp_scans.weights.push_back(1);
            temp_pt.x = scans.pts[i].x;
            temp_pt.y = scans.pts[i].y;
            temp_scans.pts.push_back(temp_pt);

            temp_map_corrs.distances.push_back(map_corrs.distances[i]);
            temp_map_corrs.ids.push_back(i);
            temp_map_corrs.weights.push_back(1);
            index = scans.ids[i];
            temp_pt.x = map_carpark.pts[index].x;
            temp_pt.y = map_carpark.pts[index].y;
            temp_map_corrs.pts.push_back(temp_pt);
        }
    }

    return temp_scans;
}

void normalize(MyPointCloud2D &scans, float sum_weights)
{
    //Normalize
    if (sum_weights > 0)
    {
        for (int n = 0; n < scans.pts.size(); n++)
        {
            scans.weights[n] /= sum_weights;
        }
    }
    else
    {
        cout << "Summe der Gewichte ist 0! Irgendwas ist schief gelaufen!" << endl;
    }
}
void Gewichtung::applyNothing(MyPointCloud2D &scans)
{
    float sum_weights = 0;

    for (int i = 0; i < scans.pts.size(); i++)
    {
        sum_weights += scans.weights[i];
    }
    normalize(scans, sum_weights);
}
void Gewichtung::divideByAllCorrespondingDistances(MyPointCloud2D &scans)
{
    float sum_weights = 0;
    for (int i = 0; i < scans.pts.size(); i++)
    {
        float sum_dist = 0;
        int index = scans.ids[i];
        for (int j = 0; j < scans.pts.size(); j++)
        {
            if ((i != j) && (index == scans.ids[j]))
            {
                sum_dist += scans.distances[j];
            }
        }
        if (sum_dist != 0)
        {
            scans.weights[i] = scans.distances[i] / sum_dist;
        }
        else
        {
            scans.weights[i] = scans.distances[i];
        }
        sum_weights += scans.weights[i];
    }
    normalize(scans, sum_weights);
}

void Gewichtung::setZeroIfTwoCorrespondings(MyPointCloud2D &scans)
{
    float sum_weights = 0;
    for (int i = 0; i < scans.pts.size(); i++)
    {
        float sum_dist = 0;
        int index = scans.ids[i];
        for (int j = 0; j < scans.pts.size(); j++)
        {
            if ((i != j) && (index == scans.ids[j]))
            {
                scans.weights[i] = 0;
                break;
            }
        }
        sum_weights += scans.weights[i];
    }
    normalize(scans, sum_weights);
}
