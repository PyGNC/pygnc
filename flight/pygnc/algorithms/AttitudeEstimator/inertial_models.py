
#From the following: 
#https://github.com/RoboticExplorationLab/non-monotonic-detumbling/blob/ddaf9219eaf03d363a48c14ae81ab7f3a28e89d7/src/magnetic_field.jl#L4

import numpy as np 
import brahe
import math 

def deg2rad(degrees):
    return degrees * (math.pi / 180.0)

def IGRF13(r_eci, epc):
    """IGRF 13 model for NED magnetic field vector in nT

    Arguments:
        r_eci: position vector from eci to sc (m)
        epc: time (Epoch)

    Returns:
        B_eci_T: magnetic field in ECI, Teslas (T)

    Comments:
        I used to use SatelliteToolbox for their IGRF, but I wrote my own
    """

    # get decimal date
    date = epc.caldate()

    year = date[0]
    #day = brahe.day_of_year(epc)
    day = epc.day_of_year()

    decimal_date = year + day / 365.2425

    # eci and ecef location
    ecef_Q_eci = brahe.rECItoECEF(epc)
    eci_Q_ecef = ecef_Q_eci.T

    # get ecef location
    r_ecef = ecef_Q_eci @ r_eci
    # println("(jl) r_ecef: ", norm(r_ecef)/1000)

    # long lat geod
    longitude, latitude, altitude = brahe.sECEFtoGEOC(r_ecef, use_degrees=True)
    # println("(jl) Long: $longitude\t Lat: $latitude\t Alt: $altitude")

    # IGRF
    # SatelliteToolbox v0.7.1
    # B_ned_nT = igrf(decimal_date, norm(r_ecef), deg2rad(latitude), deg2rad(longitude), Val(:geocentric))
    # my own IGRF function
    B_ned_nT = my_igrf_13(decimal_date, np.linalg.norm(r_ecef) / 1000, latitude, longitude, 13)
    # println("(jl) Ned: $B_ned_nT")

    # NED and ECEF DCM
    ecef_Q_ned = ecef_Q_ned_mat(deg2rad(longitude), deg2rad(latitude))

    # conver to eci
    B_eci_nT = eci_Q_ecef @ ecef_Q_ned @ B_ned_nT

    # convert from nT to T
    return B_eci_nT * 1e-9 * 1e-6


def my_igrf_13(date, alt, lat, elong, order):
    """Truncated IGRF model.

    Arguments:
    gh: truncated coefficients
    date: decimal date
    alt: radius from center of earth (km)
    lat: latitude (degrees)
    elong: east longitude (degrees)
    order: order of IGRF model
    """
    gh = np.array([-29404.8, -1450.9,  4652.5, -2499.6,  2982.0, -2991.6,   # 2020
                1677.0,  -734.6,  1363.2, -2381.2,   -82.1,  1236.2,    # 2020
                241.9,   525.7,  -543.4,   903.0,   809.5,   281.9,     # 2020
                86.3,  -158.4,  -309.4,   199.7,    48.0,  -349.7,      # 2020
                -234.3,   363.2,    47.7,   187.8,   208.3,  -140.7,    # 2020
                -121.2,  -151.2,    32.3,    13.5,    98.9,    66.0,    # 2020
                65.5,   -19.1,    72.9,    25.1,  -121.5,    52.8,      # 2020
                -36.2,   -64.5,    13.5,     8.9,   -64.7,    68.1,     # 2020
                80.6,   -76.7,   -51.5,    -8.2,   -16.9,    56.5,      # 2020
                2.2,    15.8,    23.5,     6.4,    -2.2,    -7.2,       # 2020
                -27.2,     9.8,    -1.8,    23.7,     9.7,     8.4,     # 2020
                -17.6,   -15.3,    -0.5,    12.8,   -21.1,   -11.7,     # 2020
                15.3,    14.9,    13.7,     3.6,   -16.5,    -6.9,      # 2020
                -0.3,     2.8,     5.0,     8.4,   -23.4,     2.9,      # 2020
                11.0,    -1.5,     9.8,    -1.1,    -5.1,   -13.2,      # 2020
                -6.3,     1.1,     7.8,     8.8,     0.4,    -9.3,      # 2020
                -1.4,   -11.9,     9.6,    -1.9,    -6.2,     3.4,      # 2020
                -0.1,    -0.2,     1.7,     3.6,    -0.9,     4.8,      # 2020
                0.7,    -8.6,    -0.9,    -0.1,     1.9,    -4.3,       # 2020
                1.4,    -3.4,    -2.4,    -0.1,    -3.8,    -8.8,       # 2020
                3.0,    -1.4,     0.0,    -2.5,     2.5,     2.3,       # 2020
                -0.6,    -0.9,    -0.4,     0.3,     0.6,    -0.7,      # 2020
                -0.2,    -0.1,    -1.7,     1.4,    -1.6,    -0.6,      # 2020
                -3.0,     0.2,    -2.0,     3.1,    -2.6,    -2.0,      # 2020
                -0.1,    -1.2,     0.5,     0.5,     1.3,     1.4,      # 2020
                -1.2,    -1.8,     0.7,     0.1,     0.3,     0.8,      # 2020
                0.5,    -0.2,    -0.3,     0.6,    -0.5,     0.2,       # 2020
                0.1,    -0.9,    -1.1,     0.0,    -0.3,     0.5,       # 2020
                0.1,    -0.9,    -0.9,     0.5,     0.6,     0.7,       # 2020
                1.4,    -0.3,    -0.4,     0.8,    -1.3,     0.0,       # 2020
                -0.1,     0.8,     0.3,     0.0,    -0.1,     0.4,      # 2020
                0.5,     0.1,     0.5,     0.5,    -0.4,    -0.5,       # 2020
                -0.4,    -0.4,    -0.6,                                 # 2020
                5.7,     7.4,   -25.9,   -11.0,    -7.0,   -30.2,       # 2022
                -2.1,   -22.4,     2.2,    -5.9,     6.0,     3.1,      # 2022
                -1.1,   -12.0,     0.5,    -1.2,    -1.6,    -0.1,      # 2022
                -5.9,     6.5,     5.2,     3.6,    -5.1,    -5.0,      # 2022
                -0.3,     0.5,     0.0,    -0.6,     2.5,     0.2,      # 2022
                -0.6,     1.3,     3.0,     0.9,     0.3,    -0.5,      # 2022
                -0.3,     0.0,     0.4,    -1.6,     1.3,    -1.3,      # 2022
                -1.4,     0.8,     0.0,     0.0,     0.9,     1.0,      # 2022
                -0.1,    -0.2,     0.6,     0.0,     0.6,     0.7,      # 2022
                -0.8,     0.1,    -0.2,    -0.5,    -1.1,    -0.8,      # 2022
                0.1,     0.8,     0.3,     0.0,     0.1,    -0.2,       # 2022
                -0.1,     0.6,     0.4,    -0.2,    -0.1,     0.5,      # 2022
                0.4,    -0.3,     0.3,    -0.4,    -0.1,     0.5,       # 2022
                0.4,     0.0 ]) #                        # 2022
    gh = np.hstack([gh, np.zeros(115)])

    # colat from lat
    colat = 90-lat

    # Declaration of variables
    fn    = 0
    gn    = 0
    kmx   = 0
    ll    = 0
    nc    = 0
    nmx   = 0
    x     = 0.0
    y     = 0.0
    z     = 0.0
    t     = 0.0
    tc    = 0.0

    # since date is after 2020
    t  = date - 2020
    tc = 1.0

    # gh = gh[3256:end]
    # ll  = 3255
    ll = 0
    nmx = order

    nc  = int(nmx * (nmx + 2))
    # nc = 195

    kmx = int((nmx+1) * (nmx + 2) / 2)
    # kmx = 105

    # allocate
    cl          = np.zeros(nmx)
    sl          = np.zeros(nmx)
    p           = np.zeros(kmx)
    q           = np.zeros(kmx)

    r  = alt
    ct          = np.cos(colat*np.pi/180)
    st          = np.sin(colat*np.pi/180)
    cl[0]       = np.cos(elong*np.pi/180)
    sl[0]       = np.sin(elong*np.pi/180)
    Cd = 1.0
    sd = 0.0
    l  = 1
    m  = 1
    n  = 0

    ratio = 6371.2/r
    rr    = ratio**2

    # Computation of Schmidt quasi-normal coefficients p and x(=q).
    # =============================================================

    p[0] = 1.0
    p[2] = st
    q[0] = 0.0
    q[2] = ct

    for k in range(2, kmx+1): 
        # There is no need to check bounds here. The code guarantees that
        # everything is inside the right bounds. This increased the source code
        # performance by 13%.
        if n < m:
            m  = 0
            n  = n+1
            rr = rr*ratio
            fn = n
            gn = n-1

        fm = m

        if (m == n):
            if k != 3:
                one   = np.sqrt(1 - 0.5/fm)
                j     = k - n - 2
                p[k-1]  = one*st*p[j]
                q[k-1]  = one*(st*q[j] + ct*p[j])
            # if ((k != 3)):
                cl[m-1] = cl[m-2]*cl[0] - sl[m-2]*sl[0]
                sl[m-1] = sl[m-2]*cl[0] + cl[m-2]*sl[0]
        else:
            gmm   = m**2
            one   = np.sqrt(fn**2 - gmm)
            two   = np.sqrt(gn**2 - gmm)/one
            three = (fn + gn)/one
            i     = k - n - 1
            j     = i - n + 1
            p[k-1]  = three*ct*p[i] - two*p[j]
            q[k-1]  = three*(ct*q[i] - st*p[i]) - two*q[j]

        # Synthesis of x, y, and z in geocentric coordinates.
        lm  = ll + l -1
        one = (tc*gh[lm] + t*gh[lm+nc])*rr

        if m != 0:
            two   = (tc*gh[lm+1] + t*gh[lm+nc+1])*rr
            three = one*cl[m-1] + two*sl[m-1]
            x     = x + three*q[k-1]
            z     = z - (fn + 1)*three*p[k-1]

            if st != 0:
                y = y + (one*sl[m-1] - two*cl[m-1])*fm*p[k-1]/st
            else:
                y = y + (one*sl[m-1] - two*cl[m-1])*q[k-1]*ct

            l = l + 2
        else:
            x = x + one*q[k-1]
            z = z - (fn + 1)*one*p[k-1]
            l = l + 1

        m = m + 1

    # Conversion to coordinate system specified by itype.
    # ===================================================
   
    one   = x 
    x     = x*Cd +   z*sd
    z     = z*Cd - one*sd

    return 1e6*np.array([x, y, z])


def ecef_Q_ned_mat(lon, lat):
    sp, cp = np.sin(lat), np.cos(lat)
    sl, cl = np.sin(lon), np.cos(lon)

    return np.array([  [-sp*cl, -sl, -cp*cl],
                         [-sp*sl,  cl, -cp*sl],
                         [cp,     0.0, -sp]      ])
