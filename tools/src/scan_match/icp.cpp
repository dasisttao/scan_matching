#include "icp.hpp"
#include <chrono>

void ICP::calcEqPoints(MyPointCloud2D map_corrs, MyPointCloud2D scans, Matrix2d &R, Vector2d &T)
{

    //---Find data centroid and deviations from centroid (Map)
    Vector2d map_bar;
    Vector2d scan_bar;

    for (size_t i = 0; i < map_corrs.size; i++)
    {
        map_bar(0) += map_corrs.pts[i].x * scans.weights[i];
        map_bar(1) += map_corrs.pts[i].y * scans.weights[i];
        scan_bar(0) += scans.pts[i].x * scans.weights[i];
        scan_bar(1) += scans.pts[i].y * scans.weights[i];
    }

    for (size_t i = 0; i < map_corrs.size; i++)
    {
        map_corrs.pts[i].x -= map_bar(0);
        map_corrs.pts[i].y -= map_bar(1);
        scans.pts[i].x -= scan_bar(0);
        scans.pts[i].y -= scan_bar(1);
    } //q_mark  und p_mark

    for (size_t i = 0; i < map_corrs.size; i++)
    {
        map_corrs.pts[i].x *= map_corrs.weights[i];
        map_corrs.pts[i].y *= map_corrs.weights[i];
        // scans.points(i, 0) *= map_corrs.weights[i]; //wird nicht genutzt laut Matlab code
        // scans.points(i, 1) *= map_corrs.weights[i];
    }
    // ---Multiplying map with scan points(Transform Algorithm)
    Matrix2d N;
    double a = 0, b = 0, c = 0, d = 0;

    for (size_t i = 0; i < map_corrs.size; i++)
    {
        a += scans.pts[i].x * map_corrs.pts[i].x;
        b += scans.pts[i].x * map_corrs.pts[i].y;
        c += scans.pts[i].y * map_corrs.pts[i].x;
        d += scans.pts[i].y * map_corrs.pts[i].y;
    }
    N << a, b, c, d;

    Eigen::JacobiSVD<Matrix2d> svd;
    svd.compute(N, ComputeFullU | ComputeFullV);
    double detUV = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    Matrix2d diagUV;
    diagUV << 1, 0, 0, detUV;
    R = svd.matrixV() * diagUV * svd.matrixU().transpose();
    T = map_bar - R * scan_bar;
}

MyPointCloud2D ICP::verwerfung(float filt_distance, MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark)
{
    MyPointCloud2D temp_scans;
    MyPointCloud2D temp_map_corrs;
    MyPoint temp_pt;

    //Removing correspondencies that are too far apart
    for (size_t i = 0; i < scans.distances.size(); i++)
    {
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
            int index = scans.ids[i];
            temp_pt.x = map_carpark.pts[index].x;
            temp_pt.y = map_carpark.pts[index].y;
            temp_map_corrs.pts.push_back(temp_pt);
        }
    }
    temp_scans.size = temp_scans.ids.size();
    temp_map_corrs.size = temp_map_corrs.ids.size();
    map_corrs = temp_map_corrs;
    return temp_scans;
}

float ICP::getFiltDistance(float error_before_matching, float ratio_corres_last_timestep)
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
void ICP::createPointCloud2D(PointCloud2D<float> &cloudMap, PointCloud2D<float> &cloudScan, const MyPointCloud2D &map_carpark, const MyPointCloud2D &scans)
{
    cloudMap.pts.resize(map_carpark.size);
    cloudScan.pts.resize(scans.size);
    for (int i = 0; i < map_carpark.size; i++)
    {
        cloudMap.pts[i].x = map_carpark.pts[i].x;
        cloudMap.pts[i].y = map_carpark.pts[i].y;
    }
    for (int i = 0; i < scans.size; i++)
    {
        cloudScan.pts[i].x = scans.pts[i].x;
        cloudScan.pts[i].y = scans.pts[i].y;
    }
}
MyPointCloud2D ICP::findNeigherstNeighbor(const PointCloud2D<float> &cloudMap, const PointCloud2D<float> &cloudScan, MyPointCloud2D &scans, float &distance_total_corrs, const my_kd_tree_t &index)
{
    MyPointCloud2D map_corrs;

    size_t ret_index;
    float out_dist_sqr;
    MyPoint temp_pt;
    nanoflann::KNNResultSet<float> resultSet(1);
    for (size_t idx_scan = 0; idx_scan < cloudScan.pts.size(); idx_scan++)
    {
        //Find best correspondence for current scan point
        float query[2] = {cloudScan.pts[idx_scan].x, cloudScan.pts[idx_scan].y};
        resultSet.init(&ret_index, &out_dist_sqr);
        index.findNeighbors(resultSet, &query[0], nanoflann::SearchParams(10));
        //Save id, distance and corresponding map point
        map_corrs.ids.push_back(ret_index);
        temp_pt.x = cloudMap.pts[ret_index].x;
        temp_pt.y = cloudMap.pts[ret_index].y;
        map_corrs.pts.push_back(temp_pt);
        scans.distances[idx_scan] = out_dist_sqr;
        scans.ids[idx_scan] = ret_index;
        map_corrs.distances.push_back(out_dist_sqr);
        map_corrs.weights.push_back(1);
        distance_total_corrs += out_dist_sqr; // Hier evtl nochmal quadrieren?
    }
    map_corrs.size = map_corrs.ids.size();
    return map_corrs;
}

State ICP::matchingResult(const vector<Matrix2d> &TR, const vector<Vector2d> &TT, State state)
{
    State new_state;
    Vector2d new_pos;
    Vector2d pos;
    pos << state.x, state.y;

    new_pos = TR[number_of_iterations] * pos + TT[number_of_iterations];
    new_state.x = new_pos(0);
    new_state.y = new_pos(1);
    new_state.v = state.v;
    new_state.yaw = state.yaw;
    new_state.yawr = state.yawr;
    return new_state;
}

void ICP::calcWeights(MyPointCloud2D &scans)
{
    int i = 0;
    float sum_weights = 0;
    for (i = 0; i < scans.pts.size(); i++)
    {
        float sum_dist = 0;
        int index = scans.ids[i];
        for (int j = 0; j < scans.pts.size(); j++)
        {
            if ((i != j) && (index = scans.ids[j]))
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
            scans.weights[i] = 1;
        }
        sum_weights += scans.weights[i];
    }
    //Normalize
    for (int n = 0; n < scans.pts.size(); n++)
    {

        if (sum_weights > 0)
        {
            scans.weights[n] /= sum_weights;
        }
        else
        {
            scans.weights[n] = 1;
        }
    }
}

MyPointCloud2D ICP::mainAlgorithm(const MyPointCloud2D &map_carpark, MyPointCloud2D &scans, State state, State &new_state)
{
    //---Init---
    PointCloud2D<float> cloudMap, cloudScan;
    createPointCloud2D(cloudMap, cloudScan, map_carpark, scans);
    // kd-Baum Init
    my_kd_tree_t index(2 /*dim*/, cloudMap, nanoflann::KDTreeSingleIndexAdaptorParams(10 /*max leaf*/));
    // kd-Baum erstellen
    index.buildIndex();
    MyPointCloud2D map_corrs, scans_kd;
    vector<float> error;
    error.resize(number_of_iterations);
    Matrix2d R;
    Vector2d T;
    vector<Matrix2d> TR;
    vector<Vector2d> TT;
    TR.resize(number_of_iterations + 1);
    TT.resize(number_of_iterations + 1);
    // Init Transformation Vectors & Matrixs
    TR[0] << 1, 0, 0, 1;
    TT[0] << 0, 0;
    //----Init done-----

    // ICP - Iterations - Iteration unnötig?
    for (size_t iterICP = 0; iterICP < number_of_iterations; iterICP++)
    {

        float distance_total_sqr = 0;
        if (iterICP == 0)
        {
            map_corrs = findNeigherstNeighbor(cloudMap, cloudScan, scans, distance_total_sqr, index);
            error[iterICP] = sqrt(distance_total_sqr / map_corrs.size);
            float filt_distance = getFiltDistance(error[iterICP], 0.1 /*timestep => später variable*/);
            scans_kd = verwerfung(filt_distance, map_corrs, scans, map_carpark);
        }
        else
        {
            // Im Matlab Code , aber unnötig?
            // map_corrs = findNeigherstNeighbor(cloudMap, cloudScan, scans_kd, distance_total_sqr, index);
        }
        //---Calculate Weights
        calcWeights(scans_kd);
        //---Calculate Transformation with weights
        calcEqPoints(map_corrs, scans_kd, R, T); //R & T als Referenz

        // //---Save transformation of iteration
        TR[iterICP + 1] = R * TR[iterICP];
        TT[iterICP + 1] = R * TT[iterICP] + T;
    }

    new_state = matchingResult(TR, TT, state);
    // timer.stop("kd");
    //Return scans_kd für rviz
    return scans_kd;
}
