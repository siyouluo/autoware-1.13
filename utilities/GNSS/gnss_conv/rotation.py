import numpy as np
import math


def llh_to_xyz(lad, lat, lod, lon, ele):
    M_PI = math.pi
    m_lat = (lad * lat / 60) * M_PI / 180
    m_lon = (lod * lon / 60) * M_PI / 180
    m_h = ele
    Pmo = 0.9999
    lon_deg = 26
    lon_min = 0
    lat_deg = 154
    lat_min = 0

    m_PLo = M_PI * (lat_deg) + (lat_min) / 60.0 / 180
    m_PLato = M_PI * (lon_deg) + (lon_min) / 60.0 / 180

    AW = 6378137.0
    FW = 1.0 / 298.257222101

    Pe = (math.sqrt(2.0 * FW - math.pow(FW, 2)));
    Pet = (math.sqrt(math.pow(Pe, 2) / (1.0 - math.pow(Pe, 2))));

    PA = (1.0 + 3.0 / 4.0 * math.pow(Pe, 2) + 45.0 / 64.0 * math.pow(Pe, 4) +175.0 / 256.0 * math.pow(Pe,6) + 11025.0 / 16384.0 * math.pow(Pe, 8) + 43659.0 / 65536.0 *math.pow(Pe, 10) + 693693.0 / 1048576.0 * math.pow(Pe, 12) + 19324305.0 / 29360128.0 * math.pow(Pe, 14) +4927697775.0 / 7516192768.0 * math.pow(Pe, 16))

    PB = (3.0 / 4.0 * math.pow(Pe, 2) + 15.0 / 16.0 * math.pow(Pe, 4) + 525.0 / 512.0 *math.pow(Pe,6) + 2205.0 / 2048.0 * math.pow(Pe, 8) + 72765.0 / 65536.0 * math.pow(Pe, 10) + 297297.0 / 262144.0 *math.pow(Pe, 12) + 135270135.0 / 117440512.0 * math.pow(Pe,14) + 547521975.0 / 469762048.0 * math.pow(Pe, 16))

    PC = (15.0 / 64.0 * math.pow(Pe, 4) + 105.0 / 256.0 * math.pow(Pe, 6) + 2205.0 / 4096.0 *math.pow(Pe,8) + 10395.0 / 16384.0 * math.pow(Pe, 10) + 1486485.0 / 2097152.0 * math.pow(Pe, 12) +45090045.0 / 58720256.0 * math.pow(Pe,14) + 766530765.0 / 939524096.0 * math.pow(Pe, 16))

    PD = (35.0 / 512.0 * math.pow(Pe, 6) + 315.0 / 2048.0 * math.pow(Pe, 8) + 31185.0 / 131072.0 *math.pow(Pe,10) + 165165.0 / 524288.0 * math.pow(Pe, 12) + 45090045.0 / 117440512.0 * math.pow(Pe, 14) +209053845.0 / 469762048.0 * math.pow(Pe, 16))

    PE = (315.0 / 16384.0 * math.pow(Pe, 8) + 3465.0 / 65536.0 * math.pow(Pe, 10) +99099.0 / 1048576.0 * math.pow(Pe, 12) + 4099095.0 / 29360128.0 * math.pow(Pe, 14) + 348423075.0 / 1879048192.0 *math.pow(Pe, 16))

    PF = (693.0 / 131072.0 * math.pow(Pe, 10) + 9009.0 / 524288.0 * math.pow(Pe, 12) +4099095.0 / 117440512.0 * math.pow(Pe, 14) + 26801775.0 / 469762048.0 * math.pow(Pe, 16))

    PG = (3003.0 / 2097152.0 * math.pow(Pe, 12) + 315315.0 / 58720256.0 * math.pow(Pe, 14) +11486475.0 / 939524096.0 * math.pow(Pe, 16))

    PH = (45045.0 / 117440512.0 * math.pow(Pe, 14) + 765765.0 / 469762048.0 * math.pow(Pe, 16))

    PI = (765765.0 / 7516192768.0 * math.pow(Pe, 16))

    PB1 = (AW) * (1.0 - math.pow(Pe, 2)) *PA
    PB2 = (AW) * (1.0 - math.pow(Pe, 2)) *PB / -2.0
    PB3 = (AW) * (1.0 - math.pow(Pe, 2)) *PC / 4.0
    PB4 = (AW) * (1.0 - math.pow(Pe, 2)) *PD / -6.0
    PB5 = (AW) * (1.0 - math.pow(Pe, 2)) *PE / 8.0
    PB6 = (AW) * (1.0 - math.pow(Pe, 2)) *PF / -10.0
    PB7 = (AW) * (1.0 - math.pow(Pe, 2)) *PG / 12.0
    PB8 = (AW) * (1.0 - math.pow(Pe, 2)) *PH / -14.0
    PB9 = (AW) * (1.0 - math.pow(Pe, 2)) *PI / 16.0

    PS = (PB1) * m_lat + PB2 * math.sin(2.0 * m_lat) + PB3 * math.sin(4.0 * m_lat) + PB4 * math.sin(6.0 * m_lat) + PB5 * math.sin(8.0 * m_lat) + PB6 * math.sin(10.0 * m_lat) + PB7 * math.sin(12.0 * m_lat) +PB8 * math.sin(14.0 * m_lat) + PB9 * math.sin(16.0 * m_lat)

    PSo = (PB1) * m_PLato + PB2 * math.sin(2.0 * m_PLato) + PB3 * math.sin(4.0 * m_PLato) + PB4 * math.sin(6.0 * m_PLato) + PB5 * math.sin(8.0 * m_PLato) + PB6 * math.sin(10.0 * m_PLato) + PB7 *math.sin(12.0 * m_PLato) + PB8 * math.sin(14.0 * m_PLato) + PB9 * math.sin(16.0 * m_PLato)

    PDL = (m_lon) - m_PLo
    Pt = (math.tan(m_lat))
    PW = (math.sqrt(1.0 - math.pow(Pe, 2) * math.pow(math.sin(m_lat), 2)))
    PN = (AW) / PW
    Pnn = (math.sqrt(math.pow(Pet, 2) * math.pow(math.cos(m_lat), 2)))

    m_x = math.cos (((PS - PSo) + (1.0 / 2.0) * PN * math.pow(math.cos(m_lat), 2.0) * Pt * math.pow(PDL,2.0) +(1.0 / 24.0) * PN * math.pow(math.cos(m_lat), 4) *Pt *(5.0 - math.pow(Pt, 2) + 9.0 * math.pow(Pnn, 2) + 4.0 * math.pow(Pnn, 4)) *math.pow(PDL,4) -(1.0 / 720.0) * PN * math.pow(math.cos(m_lat), 6) *Pt *(-61.0 + 58.0 * math.pow(Pt, 2) - math.pow(Pt, 4) - 270.0 * math.pow(Pnn, 2) + 330.0 *math.pow(Pt,2) * math.pow(Pnn, 2)) *math.pow(PDL, 6) - (1.0 / 40320.0) * PN * math.pow(math.cos(m_lat), 8) *Pt *(-1385.0 + 3111 * math.pow(Pt, 2) - 543 * math.pow(Pt,4) + math.pow(Pt, 6)) *math.pow(PDL, 8)) *Pmo)
    m_y =  ((PN * math.cos(m_lat) * PDL -1.0 / 6.0 * PN * math.pow(math.cos(m_lat), 3) *(-1 + math.pow(Pt, 2) - math.pow(Pnn,2)) *math.pow(PDL, 3) -1.0 / 120.0 * PN * math.pow(math.cos(m_lat), 5) *(-5.0 + 18.0 * math.pow(Pt, 2) - math.pow(Pt, 4) - 14.0 * math.pow(Pnn, 2) + 58.0 * math.pow(Pt, 2) *math.pow(Pnn,2)) *math.pow(PDL,5) -1.0 / 5040.0 * PN * math.pow(math.cos(m_lat), 7) *(-61.0 + 479.0 * math.pow(Pt, 2) - 179.0 * math.pow(Pt, 4) + math.pow(Pt, 6)) *math.pow(PDL, 7)) *Pmo);
    m_z = m_h
    return m_x, m_y, m_z


def compute_trans(p_1, p):
    c = p_1[0][1] - p_1[1][1]
    a = p[0][1] - p[1][1]
    b = p[0][0] - p[1][0]
    delta = b * math.sqrt(-c**2 + a**2 + b**2)
    cos_theta_1 = (c * a - delta) / (a**2 + b**2)
    cos_theta_2 = (c * a + delta) / (a**2 + b**2)
    if -1 <= cos_theta_1 <= 1:
        cos_theta = cos_theta_1
    else:
        cos_theta = cos_theta_2
    theta = math.acos(cos_theta)
    d_x = p[0][0] * math.cos(theta) - p[0][1] * math.sin(theta) - p_1[0][0]
    d_y = p[0][0] * math.sin(theta) + p[0][1] * math.cos(theta) - p_1[0][1]
    theta = 180 * theta / math.pi
    return theta, d_x, d_y


def conv(points_map, theta, d_x, d_y):
    theta = math.pi * theta / 180
    r_mat = np.mat(
        [
            [math.cos(theta), math.sin(theta)],
            [-math.sin(theta), math.cos(theta)]
            ]
    ).reshape((2, 2))
    d_mat = np.mat(
        [
            [d_x, d_y]
        ]
    ).reshape((1, 2))
    points_map_new = np.matmul(points_map[:, :2], r_mat) + d_mat
    return points_map_new


def out_file(poitns_map, fname):
    np.savetxt(fname, poitns_map, fmt='%.4f')


if __name__ == '__main__':
    file_name = 'autoware-200911_1.pcd'
    new_file_name = '200911_1.pcd'

    POINTS = np.loadtxt(file_name)

    P_1 = [
        [3213.70048501, 116.741940387],
        [3203.69884006, 122.263047486]
    ]
    P = [
        [4.55501317978, 0.45576736331],
        [23.7118740082, -0.540710806847]
    ]
    THETA, D_X, D_Y = compute_trans(P, P_1)

    NEW_POINTS = conv(POINTS, THETA, D_X, D_Y)

    out_file(NEW_POINTS, new_file_name)


