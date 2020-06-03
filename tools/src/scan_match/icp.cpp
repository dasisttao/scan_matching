#include "icp.hpp"
#include <chrono>

void ICP::calcEqPoints(MyPointCloud2D map_corrs, MyPointCloud2D scans, Matrix2d &R, Vector2d &T)
{

    //---Find data centroid and deviations from centroid (Map)
    Vector2d map_bar;
    Vector2d scan_bar;

    for (size_t i = 0; i < map_corrs.pts.size(); i++)
    {
        map_bar(0) += map_corrs.pts[i].x * scans.weights[i];
        map_bar(1) += map_corrs.pts[i].y * scans.weights[i];
        scan_bar(0) += scans.pts[i].x * scans.weights[i];
        scan_bar(1) += scans.pts[i].y * scans.weights[i];
    }
    for (size_t i = 0; i < map_corrs.pts.size(); i++)
    {
        map_corrs.pts[i].x -= map_bar(0);
        map_corrs.pts[i].y -= map_bar(1);
        scans.pts[i].x -= scan_bar(0);
        scans.pts[i].y -= scan_bar(1);
    } //q_mark  und p_mark

    for (size_t i = 0; i < map_corrs.pts.size(); i++)
    {
        map_corrs.pts[i].x *= map_corrs.weights[i];
        map_corrs.pts[i].y *= map_corrs.weights[i];
        // scans.points(i, 0) *= map_corrs.weights[i]; //wird nicht genutzt laut Matlab code
        // scans.points(i, 1) *= map_corrs.weights[i];
    }
    // ---Multiplying map with scan points(Transform Algorithm)
    Matrix2d N;
    double a = 0, b = 0, c = 0, d = 0;

    for (size_t i = 0; i < map_corrs.pts.size(); i++)
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

void ICP::createPointCloud2D(PointCloud2D<float> &pc, const MyPointCloud2D &my_pc)
{
    pc.pts.clear();
    pc.pts.resize(my_pc.pts.size());

    for (int i = 0; i < my_pc.pts.size(); i++)
    {
        pc.pts[i].x = my_pc.pts[i].x;
        pc.pts[i].y = my_pc.pts[i].y;
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
    return map_corrs;
}

State ICP::matchingResult(const vector<Matrix2d> &TR, const vector<Vector2d> &TT, State state, Matrix2d &rotM)
{
    State new_state;
    Vector2d new_pos;
    Vector2d pos;
    Matrix2d new_rot;
    pos << state.x, state.y;

    new_pos = TR[number_of_iterations] * pos + TT[number_of_iterations];
    new_rot = TR[number_of_iterations] * rotM;
    new_state.x = new_pos(0);
    new_state.y = new_pos(1);
    new_state.v = state.v;
    if (abs(state.yaw + acos(TR[number_of_iterations](0, 0))) < M_PI)
    {
        new_state.yaw = -acos(new_rot(0, 0));
    }
    else
    {
        new_state.yaw = 2 * M_PI - acos(new_rot(0, 0));
    }

    if (state.yaw > 0)
    {
        new_state.yaw = -new_state.yaw;
    }
    new_state.yaw = atan2(sin(new_state.yaw), cos(new_state.yaw));
    new_state.yawr = state.yawr;
    return new_state;
}

void transformLast(Matrix2d TR, Vector2d TT, MyPointCloud2D &scans_kd)
{
    for (int i = 0; i < scans_kd.pts.size(); i++)
    {
        scans_kd.pts[i].x = TR(0, 0) * scans_kd.pts[i].x + TR(0, 1) * scans_kd.pts[i].y + TT(0);
        scans_kd.pts[i].y = TR(1, 0) * scans_kd.pts[i].x + TR(1, 1) * scans_kd.pts[i].y + TT(1);
    }
}

MyPointCloud2D ICP::mainAlgorithm(const MyPointCloud2D &map_carpark, MyPointCloud2D &scans, State state, State &new_state, Matrix2d &rotM)
{
    //---Init---
    PointCloud2D<float> cloudMap, cloudScan;
    createPointCloud2D(cloudMap, map_carpark);
    createPointCloud2D(cloudScan, scans);
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
            error[iterICP] = sqrt(distance_total_sqr / map_corrs.pts.size());
            scans_kd = verwerfung.selectVewerfung(verwerfungs_methode, map_corrs, scans, map_carpark, state);
            createPointCloud2D(cloudScan, scans_kd);
        }
        else
        {
            // Im Matlab Code , aber unnötig?
            map_corrs = findNeigherstNeighbor(cloudMap, cloudScan, scans_kd, distance_total_sqr, index);
            error[iterICP] = sqrt(distance_total_sqr / map_corrs.pts.size());
            scans_kd = verwerfung.selectVewerfung(verwerfungs_methode, map_corrs, scans, map_carpark, state);
            createPointCloud2D(cloudScan, scans_kd);
        }

        //---Calculate Weights
        gewichtung.selectGewichtung(gewichtungs_methode, scans_kd);

        //---Calculate Transformation with weights
        calcEqPoints(map_corrs, scans_kd, R, T); //R & T als Referenz

        // //---Save transformation of iteration
        TR[iterICP + 1] = R * TR[iterICP];
        TT[iterICP + 1] = R * TT[iterICP] + T;

        // LastTransform (Matlab)
        transformLast(TR[iterICP + 1], TT[iterICP + 1], scans_kd);
    }

    new_state = matchingResult(TR, TT, state, rotM);
    //Return scans_kd für rviz
    return scans_kd;
}
