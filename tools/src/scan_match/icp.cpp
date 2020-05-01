#include "icp.hpp"
#include <chrono>

void ICP::calcEqPoints(MyPointCloud2D map_corrs, MyPointCloud2D scans, Matrix2f &R, Vector2f &T)
{

    //---Find data centroid and deviations from centroid (Map)
    Vector2f map_bar;
    Vector2f scan_bar;
    for (size_t i = 0; i < map_corrs.size; i++)
    {
        map_bar(0) += map_corrs.pts[i].x * map_corrs.weights[i];
        map_bar(1) += map_corrs.pts[i].y * map_corrs.weights[i];
        scan_bar(0) += scans.pts[i].x * map_corrs.weights[i];
        scan_bar(1) += scans.pts[i].y * map_corrs.weights[i];
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
    //---Multiplying map with scan points (Transform Algorithm)
    Matrix2f N;
    float a, b, c, d = 0;
    for (size_t i = 0; i < map_corrs.size; i++)
    {
        a += scans.pts[i].x * map_corrs.pts[i].x;
        b += scans.pts[i].x * map_corrs.pts[i].y;
        c += scans.pts[i].y * map_corrs.pts[i].x;
        d += scans.pts[i].y * map_corrs.pts[i].y;
    }
    N << a, b, c, d;
    Eigen::JacobiSVD<Matrix2f> svd;
    svd.compute(N, ComputeFullU | ComputeFullV);
    float detUV = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    Matrix2f diagUV;
    diagUV << 1, 0, 0, detUV;
    R = svd.matrixV() * diagUV * svd.matrixU().transpose();
    T = map_bar - R * scan_bar;
}

MyPointCloud2D ICP::verwerfung(float filt_distance, MyPointCloud2D &map_corrs, const MyPointCloud2D &scans)
{
    MyPointCloud2D temp_map_corrs;
    MyPointCloud2D temp_scans;
    MyPoint temp_pt;
    //Removing correspondencies that are too far apart
    for (size_t i = 0; i < map_corrs.distances.size(); i++)
    {
        if (map_corrs.distances[i] < filt_distance)
        {
            temp_map_corrs.distances.push_back(map_corrs.distances[i]);
            temp_map_corrs.ids.push_back(map_corrs.ids[i]);
            temp_map_corrs.weights.push_back(1);
            temp_pt.x = map_corrs.pts[i].x;
            temp_pt.y = map_corrs.pts[i].y;
            temp_map_corrs.pts.push_back(temp_pt);

            temp_scans.distances.push_back(map_corrs.distances[i]);
            temp_scans.ids.push_back(scans.ids[map_corrs.ids[i]]);
            temp_scans.weights.push_back(1);
            temp_pt.x = scans.pts[i].x;
            temp_pt.y = scans.pts[i].y;
            temp_scans.pts.push_back(temp_pt);
        }
    }
    temp_map_corrs.size = temp_map_corrs.ids.size();
    temp_scans.size = temp_scans.ids.size();

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
MyPointCloud2D ICP::findNeigherstNeighbor(const PointCloud2D<float> &cloudMap, const PointCloud2D<float> &cloudScan, float &distance_total_corrs, const my_kd_tree_t &index)
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
        map_corrs.distances.push_back(out_dist_sqr);
        map_corrs.weights.push_back(1);
        distance_total_corrs += out_dist_sqr; // Hier evtl nochmal quadrieren?
    }
    map_corrs.size = map_corrs.ids.size();
    return map_corrs;
}

void ICP::mainAlgorithm(const MyPointCloud2D &map_carpark, const MyPointCloud2D &scans)
{
    //---Init---
    PointCloud2D<float> cloudMap, cloudScan;
    createPointCloud2D(cloudMap, cloudScan, map_carpark, scans);
    // kd-Baum Init
    my_kd_tree_t index(2 /*dim*/, cloudMap, nanoflann::KDTreeSingleIndexAdaptorParams(10 /*max leaf*/));
    // kd-Baum erstellen
    index.buildIndex();
    timer.start();
    MyPointCloud2D map_corrs, scans_kd;
    vector<float> error;
    error.resize(number_of_iterations);
    Matrix2f R;
    Vector2f T;
    vector<Matrix2f> TR;
    vector<Vector2f> TT;
    TR.resize(number_of_iterations + 1);
    TT.resize(number_of_iterations + 1);
    // Init Transformation Vectors & Matrixs
    for (Vector2f &vec : TT)
    {
        vec << 0, 0;
    }
    for (Matrix2f &mat : TR)
    {
        mat << 1, 0, 0, 1;
    }
    //----Init done-----

    // ICP - Iterations
    for (size_t iterICP = 0; iterICP < number_of_iterations; iterICP++)
    {

        float distance_total_sqr = 0;
        if (iterICP == 0)
        {
            map_corrs = findNeigherstNeighbor(cloudMap, cloudScan, distance_total_sqr, index);
            error[iterICP] = sqrt(distance_total_sqr / map_corrs.size);
            float filt_distance = getFiltDistance(error[iterICP], 0.1 /*timestep => später variable*/);
            scans_kd = verwerfung(filt_distance, map_corrs, scans);
        }
        else
        {
            map_corrs = findNeigherstNeighbor(cloudMap, cloudScan, distance_total_sqr, index);
            scans_kd = scans;
        }
        // //---Calculate Weights
        // calcWeights(kd_scans);
        //---Calculate Transformation with weights

        calcEqPoints(map_corrs, scans_kd, R, T); //R & T als Referenz
        // //---Save transformation of iteration
        TR[iterICP + 1] = R * TR[iterICP];
        TT[iterICP + 1] = R * TT[iterICP] + T;
        // //Last Transformation
        // MatrixXf pt_filtered;
        // pt_filtered = transformationFilteredScans(scan_points_filtered, TR[IterICP + 1], TT[IterICP + 1]);
        // //New Transformaton
        // MatrixXf pt_all;
        // pt_all = transformationAllScans(scan_points, TR[IterICP + 1], TT[IterICP + 1]);
        // //Error for this iteration
        // vector<NearestScanPoint> kd_scans_check;
        // MyPointCloud2D<float> cloudMap_check;
        // MyPointCloud2D<float> cloudScan_check;
        // ScanPoints pt_all_sp = Matrix2fToScanpoints(pt_all);
        // createPointCloud2D(cloudMap_check, cloudScan_check, map_park.size(), scan_points.size(), map_park, pt_all_sp);
        // kd_scans_check = findNeigherstNeighbor(cloudMap, cloudScan, distance_total_corrs_sqr, resultSet, index);
    }
    timer.stop("kd");
    // for (int i = 0; i < number_of_iterations + 1; i++)
    // {
    //     // cout << "TR: " << TR[i] << endl;
    //     cout << "TT: " << TT[i] << endl;
    // }
}
