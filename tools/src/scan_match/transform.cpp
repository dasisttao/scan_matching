#include <scan_match/transform.hpp>

std::vector<double> CoordTransform::Local2UTM(const std::vector<double> &sv)
{
	double x = sv[0];
	double y = sv[1];
	double yaw = sv[2];

	yaw = -yaw + M_PI / 2;

	while (yaw > 2 * M_PI)
		yaw -= 2. * M_PI;
	while (yaw < -0)
		yaw += 2. * M_PI;

	double xutm1 = 606167.51;
	double xutm2 = 606198.75;
	double yutm1 = 5797146.69;
	double yutm2 = 5797156.54;

	double xworld1 = 0.2;
	double xworld2 = 0.7056;
	double yworld1 = -0.2;
	double yworld2 = 32.8302;

	double dx = xutm1 - xworld1;
	double dy = yutm1 - yworld1;
	double arutm = (yutm1 - yutm2) / (xutm1 - xutm2);
	double arworld = (yworld1 - yworld2) / (xworld1 - xworld2); // Teilen durch 0!?!?
	double mu = atan(arutm);
	double ml = atan(arworld);

	double phi;
	if (ml < 0 && mu > 0)
		phi = -(M_PI + ml - mu);
	else if (ml < 0 && mu < 0)
		phi = -(-ml + mu);
	else
		phi = -(ml - mu);

	double yawA = yaw - phi;
	while (yawA > 2 * M_PI)
		yawA -= 2. * M_PI;
	while (yawA < -0)
		yawA += 2. * M_PI;

	double xA2 = x + xutm1 - xworld1;
	double yA2 = y + yutm1 - yworld1;

	std::vector<double> svutm = {xA2, yA2, yawA};
	return svutm;
}

// @Transform from UTM coordinate to Local
// @@input: sv = [ UTM_LONG, UTM_LAT, UTM_YAW ]
// @@output: [Local_x, Local_y, Local_yaw]
std::vector<double> CoordTransform::UTM2Local(const std::vector<double> &sv)
{
	double x = sv[0];
	double y = sv[1];
	double yaw = sv[2];

	yaw = -yaw + M_PI / 2;

	while (yaw > M_PI)
		yaw -= 2. * M_PI;
	while (yaw < -M_PI)
		yaw += 2. * M_PI;

	double xutm1 = 606167.51;
	double xutm2 = 606198.75;
	double yutm1 = 5797146.69;
	double yutm2 = 5797156.54;

	double xworld1 = 0.2;
	double xworld2 = 0.7056;
	double yworld1 = -0.2;
	double yworld2 = 32.8302;

	double dx = xutm1 - xworld1;
	double dy = yutm1 - yworld1;
	double arutm = (yutm1 - yutm2) / (xutm1 - xutm2);
	double arworld = (yworld1 - yworld2) / (xworld1 - xworld2);
	double mu = atan(arutm);
	double ml = atan(arworld);

	double phi;
	if (ml < 0 && mu > 0)
		phi = -(M_PI + ml - mu);
	else if (ml < 0 && mu < 0)
		phi = -(-ml + mu);
	else
		phi = -(ml - mu);

	phi = 0;
	MatrixXd DM(2, 2);
	DM << cos(phi), sin(phi), -sin(phi), cos(phi);

	MatrixXd PW(2, 1);
	PW << x, y;

	MatrixXd PW0(2, 1);
	PW0 << dx, dy;

	MatrixXd PW_A = DM * PW - DM * PW0;

	double yawA = yaw - phi;
	while (yawA > M_PI)
		yawA -= 2. * M_PI;
	while (yawA < -M_PI)
		yawA += 2. * M_PI;

	std::vector<double> svutm = {PW_A(0), PW_A(1), yawA};
	return svutm;
}

// @Transform wgs84 coordinates to utm
// @@input: deg: WGS84_LONG, WGS84_LAT
// @@output: utm: UTM_LONG, UTM_LAT, UTM_ZONE_No, UTM_ZONE_Char(ASCii)
bool CoordTransform::wgs84deg2utm(const std::vector<double> &deg, std::vector<double> &utm)
{
	if (deg.size() != 2)
		return false;
	double Lon = deg[0];
	double Lat = deg[1];
	double ella = 6378137;
	double ellf = 1 / 298.257223563;
	double ellb = 6378137 * (1 - 1 / 298.257223563);

	int ID[2] = {0, 0};
	int L0;

	if (Lat >= 84)
	{
		if (Lon < 0 && Lat < 90)
		{
			ID[0] = 0;
			ID[1] = 25;
			L0 = 0;
		}
		else
		{
			ID[0] = 0;
			ID[1] = 26;
			L0 = 0;
		}
	}
	else if (Lat < -80)
	{
		if (Lon < 0 && Lat > -90)
		{
			ID[0] = 0;
			ID[1] = 1;
			L0 = 0;
		}
		else
		{
			ID[0] = 0;
			ID[1] = 2;
			L0 = 0;
		}
	}
	else if (Lat >= 56 && Lat <= 64 && Lon >= 0 && Lon < 3)
	{
		ID[0] = 31;
		ID[1] = 22;
		L0 = 3;
	}
	else if (Lat >= 56 && Lat <= 64 && Lon >= 3 && Lon < 12)
	{
		ID[0] = 32;
		ID[1] = 22;
		L0 = 9;
	}
	else if (Lat >= 72 && Lat <= 84 && Lon >= 0 && Lon < 9)
	{
		ID[0] = 31;
		ID[1] = 24;
		L0 = 3;
	}
	else if (Lat >= 72 && Lat <= 84 && Lon >= 9 && Lon < 21)
	{
		ID[0] = 33;
		ID[1] = 24;
		L0 = 15;
	}
	else if (Lat >= 72 && Lat <= 84 && Lon >= 21 && Lon < 33)
	{
		ID[0] = 35;
		ID[1] = 24;
		L0 = 27;
	}
	else if (Lat >= 72 && Lat <= 84 && Lon >= 33 && Lon < 42)
	{
		ID[0] = 37;
		ID[1] = 24;
		L0 = 39;
	}
	else
	{
		ID[0] = floor(Lon / 6) + 31;
		if (Lat >= 72)
			ID[1] = 24;
		else
		{
			ID[1] = floor(Lat / 8) + 13;
			if (ID[1] >= 9)
				ID[1] = ID[1] + 1;
			if (ID[1] >= 15)
				ID[1] = ID[1] + 1;
		}
		L0 = ID[0] * 6 - 183;
	}

	double PROs[2] = {0, 0};
	double rho = 180.0 / M_PI;

	double e2 = (pow(ella, 2) - pow(ellb, 2)) / pow(ella, 2);
	double es2 = (pow(ella, 2) - pow(ellb, 2)) / pow(ellb, 2);

	if (ID[0] > 0)
	{
		double B = Lat / rho;
		double L = (Lon - L0) / rho;

		double m0 = 0.9996;

		double V = sqrt(1 + es2 * pow(cos(B), 2));
		double eta = sqrt(es2 * pow(cos(B), 2));

		double Bf = atan(tan(B) / cos(V * L) * (1 + pow(eta, 2) / 6 * (1 - 3 * pow(sin(B), 2)) * pow(L, 4)));
		double Vf = sqrt(1 + es2 * pow(cos(Bf), 2));
		double etaf = sqrt(es2 * pow(cos(Bf), 2));
		double n = (ella - ellb) / (ella + ellb);

		double r1 = (1.0 + n * n / 4.0 + pow(n, 4) / 64.0) * Bf;
		double r2 = 3.0 / 2.0 * n * (1.0 - n * n / 8.0) * sin(2 * Bf);
		double r3 = 15.0 / 16.0 * n * n * (1.0 - n * n / 4) * sin(4 * Bf);
		double r4 = 35.0 / 48.0 * pow(n, 3) * sin(6 * Bf);
		double r5 = 315.0 / 512.0 * pow(n, 4) * sin(8 * Bf);

		PROs[1] = ella / (1 + n) * (r1 - r2 + r3 - r4 + r5) * m0;

		if (B < 0)
			PROs[1] = PROs[1] + 10e6;

		double ys = asinh(tan(L) * cos(Bf) / Vf * (1 + etaf * etaf * L * L * pow(cos(Bf), 2) * (etaf * etaf / 6 + L * L / 10)));
		double y = m0 * ella * ella / ellb * ys;

		PROs[0] = y + 5e5;
	}

	if (ID[0] == 0)
	{
		double m0 = 0.994;

		if (ID[1] > 24)
		{
			double B = Lat / rho;
			double L = Lon / rho;
			double C0 = 2 * ella / sqrt(1 - e2) * pow(((1 - sqrt(e2)) / (1 + sqrt(e2))), sqrt(e2) / 2);
			double tanz2 = pow(((1 + sqrt(e2) * sin(B)) / (1 - sqrt(e2) * sin(B))), (sqrt(e2) / 2)) * tan(M_PI / 4 - B / 2);
			double R = m0 * C0 * tanz2;

			PROs[0] = 2e6 + R * sin(L);
			PROs[1] = 2e6 - R * cos(L);
		}
		else if (ID[1] < 3)
		{
			double B = -Lat / rho;
			double L = Lon / rho;
			double C0 = 2 * ella / sqrt(1 - e2) * pow(((1 - sqrt(e2)) / (1 + sqrt(e2))), sqrt(e2) / 2);
			double tanz2 = pow(((1 + sqrt(e2) * sin(B)) / (1 - sqrt(e2) * sin(B))), (sqrt(e2) / 2)) * tan(M_PI / 4 - B / 2);
			double R = m0 * C0 * tanz2;

			PROs[0] = 2e6 + R * sin(L);
			PROs[1] = 2e6 + R * cos(L);
		}
	}

	utm.resize(4);
	utm[0] = PROs[0];
	utm[1] = PROs[1];
	utm[2] = ID[0];
	utm[3] = ID[1] + 64;

	return true;
}

// @Transform wgs84 orientation to utm
// @@input: deg: WGS84_LONG, WGS84_LAT, UTM_ZONE_No, WGS84_YAW
// @@output: UTM_YAW
double CoordTransform::wgs84yaw2utm(const std::vector<double> &deg)
{
	// if (deg.size() != 4)
	double Lon = deg[0];
	double Lat = deg[1];
	double yaw = deg[2];
	int utm_zone = deg[3];

	double long_centMeridian = ((utm_zone - 30) * 6 - 3) / 180.0 * M_PI;

	double long_rad = Lon * M_PI / 180.0;
	double lat_rad = Lat * M_PI / 180.0;

	double meridiankonvergenz_deg_utm = atan(tan(long_centMeridian - long_rad) * sin(lat_rad)) * 180.0 / M_PI;

	double yaw_utm = yaw + meridiankonvergenz_deg_utm;

	while (yaw_utm > 360)
		yaw_utm -= 360;
	while (yaw_utm < 0)
		yaw_utm += 360;

	return yaw_utm;
}

// @Transform wgs84 coordinates and orientation to utm
// @@input: deg: WGS84_LONG, WGS84_LAT, WGS84_YAW
// @@output: utm: UTM_LONG, UTM_LAT, UTM_YAW
bool CoordTransform::wgs84Toutm(const std::vector<double> &deg, std::vector<double> &utm)
{
	if (deg.size() != 3)
		return false;
	double Lon = deg[0];
	double Lat = deg[1];
	double yaw = deg[2];

	std::vector<double> wsg_Lon_Lat = {Lon, Lat};
	std::vector<double> utm_Lon_Lat;

	wgs84deg2utm(wsg_Lon_Lat, utm_Lon_Lat);

	std::vector<double> wsg_Lon_Lat_Yaw = {Lon, Lat, yaw, utm_Lon_Lat[2]};
	double utm_yaw = wgs84yaw2utm(wsg_Lon_Lat_Yaw);

	double AdmaOffsetX = 1.5;	 //m
	double AdmaOffsetY = -0.450; //m
	double AdmaOffsetZ = 1.53;	 //m

	//@ transform from ADMA-Coordinate to Vehicle-Coordinate.
	double utm_yaw_rad = utm_yaw / 180.0 * M_PI;
	utm_Lon_Lat[0] = utm_Lon_Lat[0] - sin(utm_yaw_rad) * AdmaOffsetX + cos(utm_yaw_rad) * AdmaOffsetY;
	utm_Lon_Lat[1] = utm_Lon_Lat[1] - cos(utm_yaw_rad) * AdmaOffsetX - sin(utm_yaw_rad) * AdmaOffsetY;

	utm.resize(3);
	utm[0] = utm_Lon_Lat[0];
	utm[1] = utm_Lon_Lat[1];
	utm[2] = utm_yaw;

	return true;
}

vector<double> CoordTransform::getLocalPoseFromGPS(const gpsData::ConstPtr &gps_data, geometry_msgs::PoseStamped &gps_pose)
{
	vector<double> wgs84_pt;
	vector<double> utm_pt;
	vector<double> utm_local_pt;
	//Transform from WGS84 to UTM
	wgs84_pt.push_back(gps_data->ins_pos_abs.In_Long);
	wgs84_pt.push_back(gps_data->ins_pos_abs.In_Lat);
	wgs84_pt.push_back(gps_data->ins_angle_gps_course.Angle_Yaw);
	wgs84_pt.push_back(32);
	//Adjust for Meridian Convergence_
	double utm_yaw = wgs84yaw2utm(wgs84_pt);
	wgs84_pt.pop_back();
	wgs84Toutm(wgs84_pt, utm_pt);

	//Transform from UTM to Local
	utm_local_pt = UTM2Local(utm_pt);
	utm_local_pt[2] = -(utm_yaw - 90) * M_PI / 180.0;

	//Set Pose for rviz (debugging)
	gps_pose.pose.position.x = utm_local_pt[0];
	gps_pose.pose.position.y = utm_local_pt[1];
	gps_pose.pose.position.z = 1.1;
	gps_pose.pose.orientation.z = 0;
	gps_pose.pose.orientation.y = 1;
	gps_pose.pose.orientation.x = 0;
	gps_pose.pose.orientation.w = 1;

	return utm_local_pt;
}
