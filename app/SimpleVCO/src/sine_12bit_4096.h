#pragma once
#include <Arduino.h>
const uint16_t sine_12bit_4096[4096] = {
2048,2051,2054,2057,2060,2063,2066,2069,2073,2076,2079,2082,2085,2088,2091,2095,2098,2101,2104,2107,2110,2113,2117,2120,2123,2126,2129,2132,2135,2139,2142,2145,2148,2151,2154,2157,2161,2164,2167,2170,2173,2176,2179,2182,2186,2189,2192,2195,2198,2201,2204,2208,2211,2214,2217,2220,2223,2226,2229,2233,2236,2239,2242,2245,2248,2251,2254,2258,2261,2264,2267,2270,2273,2276,2279,2283,2286,2289,2292,2295,2298,2301,2304,2307,2311,2314,2317,2320,2323,2326,2329,2332,2335,2339,2342,2345,2348,2351,2354,2357,2360,2363,2367,2370,2373,2376,2379,2382,2385,2388,2391,2394,2398,2401,2404,2407,2410,2413,2416,2419,2422,2425,2428,2432,2435,2438,2441,2444,2447,2450,2453,2456,2459,2462,2465,2468,2472,2475,2478,2481,2484,2487,2490,2493,2496,2499,2502,2505,2508,2511,2514,2518,2521,2524,2527,2530,2533,2536,2539,2542,2545,2548,2551,2554,2557,2560,2563,2566,2569,2572,2575,2578,2581,2585,2588,2591,2594,2597,2600,2603,2606,2609,2612,2615,2618,2621,2624,2627,2630,2633,2636,2639,2642,2645,2648,2651,2654,2657,2660,2663,2666,2669,2672,2675,2678,2681,2684,2687,2690,2693,2696,2699,2702,2705,2708,2711,2714,2717,2720,2722,2725,2728,2731,2734,2737,2740,2743,2746,2749,2752,2755,2758,2761,2764,2767,2770,2773,2776,2779,2781,2784,2787,2790,2793,2796,2799,2802,2805,2808,2811,2814,2817,2819,2822,2825,2828,2831,2834,2837,2840,2843,2846,2848,2851,2854,2857,2860,2863,2866,2869,2871,2874,2877,2880,2883,2886,2889,2892,2894,2897,2900,2903,2906,2909,2912,2914,2917,2920,2923,2926,2929,2931,2934,2937,2940,2943,2946,2948,2951,2954,2957,2960,2962,2965,2968,2971,2974,2976,2979,2982,2985,2988,2990,2993,2996,2999,3002,3004,3007,3010,3013,3015,3018,3021,3024,3026,3029,3032,3035,3038,3040,3043,3046,3048,3051,3054,3057,3059,3062,3065,3068,3070,3073,3076,3078,3081,3084,3087,3089,3092,3095,3097,3100,3103,3105,3108,3111,3114,3116,3119,3122,3124,3127,3130,3132,3135,3138,3140,3143,3146,3148,3151,3153,3156,3159,3161,3164,3167,3169,3172,3175,3177,3180,3182,3185,3188,3190,3193,3195,3198,3201,3203,3206,3208,3211,3214,3216,3219,3221,3224,3226,3229,3232,3234,3237,3239,3242,3244,3247,3249,3252,3255,3257,3260,3262,3265,3267,3270,3272,3275,3277,3280,3282,3285,3287,3290,3292,3295,3297,3300,3302,3305,3307,3310,3312,3315,3317,3319,3322,3324,3327,3329,3332,3334,3337,3339,3342,3344,3346,3349,3351,3354,3356,3358,3361,3363,3366,3368,3370,3373,3375,3378,3380,3382,3385,3387,3390,3392,3394,3397,3399,3401,3404,3406,3408,3411,3413,3415,3418,3420,3422,3425,3427,3429,3432,3434,3436,3439,3441,3443,3446,3448,3450,3452,3455,3457,3459,3462,3464,3466,3468,3471,3473,3475,3477,3480,3482,3484,3486,3489,3491,3493,3495,3497,3500,3502,3504,3506,3508,3511,3513,3515,3517,3519,3522,3524,3526,3528,3530,3532,3535,3537,3539,3541,3543,3545,3548,3550,3552,3554,3556,3558,3560,3562,3565,3567,3569,3571,3573,3575,3577,3579,3581,3583,3585,3587,3590,3592,3594,3596,3598,3600,3602,3604,3606,3608,3610,3612,3614,3616,3618,3620,3622,3624,3626,3628,3630,3632,3634,3636,3638,3640,3642,3644,3646,3648,3650,3652,3654,3656,3658,3660,3662,3663,3665,3667,3669,3671,3673,3675,3677,3679,3681,3683,3684,3686,3688,3690,3692,3694,3696,3698,3699,3701,3703,3705,3707,3709,3710,3712,3714,3716,3718,3720,3721,3723,3725,3727,3729,3730,3732,3734,3736,3737,3739,3741,3743,3745,3746,3748,3750,3752,3753,3755,3757,3758,3760,3762,3764,3765,3767,3769,3770,3772,3774,3776,3777,3779,3781,3782,3784,3786,3787,3789,3790,3792,3794,3795,3797,3799,3800,3802,3804,3805,3807,3808,3810,3812,3813,3815,3816,3818,3819,3821,3823,3824,3826,3827,3829,3830,3832,3833,3835,3837,3838,3840,3841,3843,3844,3846,3847,3849,3850,3852,3853,3855,3856,3857,3859,3860,3862,3863,3865,3866,3868,3869,3870,3872,3873,3875,3876,3878,3879,3880,3882,3883,3885,3886,3887,3889,3890,3891,3893,3894,3896,3897,3898,3900,3901,3902,3904,3905,3906,3907,3909,3910,3911,3913,3914,3915,3917,3918,3919,3920,3922,3923,3924,3925,3927,3928,3929,3930,3932,3933,3934,3935,3936,3938,3939,3940,3941,3942,3944,3945,3946,3947,3948,3950,3951,3952,3953,3954,3955,3956,3958,3959,3960,3961,3962,3963,3964,3965,3966,3968,3969,3970,3971,3972,3973,3974,3975,3976,3977,3978,3979,3980,3981,3982,3983,3984,3985,3986,3987,3988,3989,3990,3991,3992,3993,3994,3995,3996,3997,3998,3999,4000,4001,4002,4003,4004,4005,4006,4007,4007,4008,4009,4010,4011,4012,4013,4014,4015,4015,4016,4017,4018,4019,4020,4021,4021,4022,4023,4024,4025,4025,4026,4027,4028,4029,4029,4030,4031,4032,4033,4033,4034,4035,4036,4036,4037,4038,4039,4039,4040,4041,4041,4042,4043,4044,4044,4045,4046,4046,4047,4048,4048,4049,4050,4050,4051,4052,4052,4053,4053,4054,4055,4055,4056,4057,4057,4058,4058,4059,4059,4060,4061,4061,4062,4062,4063,4063,4064,4064,4065,4066,4066,4067,4067,4068,4068,4069,4069,4070,4070,4071,4071,4072,4072,4072,4073,4073,4074,4074,4075,4075,4076,4076,4076,4077,4077,4078,4078,4078,4079,4079,4080,4080,4080,4081,4081,4081,4082,4082,4082,4083,4083,4083,4084,4084,4084,4085,4085,4085,4086,4086,4086,4086,4087,4087,4087,4088,4088,4088,4088,4089,4089,4089,4089,4089,4090,4090,4090,4090,4091,4091,4091,4091,4091,4091,4092,4092,4092,4092,4092,4092,4093,4093,4093,4093,4093,4093,4093,4093,4093,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4093,4093,4093,4093,4093,4093,4093,4093,4092,4092,4092,4092,4092,4092,4092,4091,4091,4091,4091,4091,4090,4090,4090,4090,4090,4089,4089,4089,4089,4088,4088,4088,4088,4087,4087,4087,4087,4086,4086,4086,4085,4085,4085,4085,4084,4084,4084,4083,4083,4083,4082,4082,4082,4081,4081,4081,4080,4080,4079,4079,4079,4078,4078,4077,4077,4077,4076,4076,4075,4075,4074,4074,4074,4073,4073,4072,4072,4071,4071,4070,4070,4069,4069,4068,4068,4067,4067,4066,4066,4065,4065,4064,4064,4063,4063,4062,4061,4061,4060,4060,4059,4059,4058,4057,4057,4056,4056,4055,4054,4054,4053,4053,4052,4051,4051,4050,4049,4049,4048,4047,4047,4046,4045,4045,4044,4043,4042,4042,4041,4040,4040,4039,4038,4037,4037,4036,4035,4034,4034,4033,4032,4031,4031,4030,4029,4028,4027,4027,4026,4025,4024,4023,4023,4022,4021,4020,4019,4018,4018,4017,4016,4015,4014,4013,4012,4011,4011,4010,4009,4008,4007,4006,4005,4004,4003,4002,4001,4001,4000,3999,3998,3997,3996,3995,3994,3993,3992,3991,3990,3989,3988,3987,3986,3985,3984,3983,3982,3981,3980,3979,3978,3977,3976,3975,3973,3972,3971,3970,3969,3968,3967,3966,3965,3964,3963,3961,3960,3959,3958,3957,3956,3955,3954,3952,3951,3950,3949,3948,3947,3945,3944,3943,3942,3941,3940,3938,3937,3936,3935,3933,3932,3931,3930,3929,3927,3926,3925,3924,3922,3921,3920,3918,3917,3916,3915,3913,3912,3911,3909,3908,3907,3906,3904,3903,3902,3900,3899,3898,3896,3895,3893,3892,3891,3889,3888,3887,3885,3884,3882,3881,3880,3878,3877,3875,3874,3873,3871,3870,3868,3867,3865,3864,3863,3861,3860,3858,3857,3855,3854,3852,3851,3849,3848,3846,3845,3843,3842,3840,3839,3837,3836,3834,3833,3831,3830,3828,3827,3825,3823,3822,3820,3819,3817,3816,3814,3812,3811,3809,3808,3806,3804,3803,3801,3799,3798,3796,3795,3793,3791,3790,3788,3786,3785,3783,3781,3780,3778,3776,3775,3773,3771,3770,3768,3766,3764,3763,3761,3759,3758,3756,3754,3752,3751,3749,3747,3745,3744,3742,3740,3738,3737,3735,3733,3731,3729,3728,3726,3724,3722,3720,3719,3717,3715,3713,3711,3710,3708,3706,3704,3702,3700,3698,3697,3695,3693,3691,3689,3687,3685,3683,3682,3680,3678,3676,3674,3672,3670,3668,3666,3664,3662,3661,3659,3657,3655,3653,3651,3649,3647,3645,3643,3641,3639,3637,3635,3633,3631,3629,3627,3625,3623,3621,3619,3617,3615,3613,3611,3609,3607,3605,3603,3601,3599,3597,3595,3593,3591,3589,3586,3584,3582,3580,3578,3576,3574,3572,3570,3568,3566,3563,3561,3559,3557,3555,3553,3551,3549,3546,3544,3542,3540,3538,3536,3534,3531,3529,3527,3525,3523,3521,3518,3516,3514,3512,3510,3507,3505,3503,3501,3499,3496,3494,3492,3490,3487,3485,3483,3481,3478,3476,3474,3472,3469,3467,3465,3463,3460,3458,3456,3454,3451,3449,3447,3444,3442,3440,3438,3435,3433,3431,3428,3426,3424,3421,3419,3417,3414,3412,3410,3407,3405,3403,3400,3398,3395,3393,3391,3388,3386,3384,3381,3379,3376,3374,3372,3369,3367,3364,3362,3360,3357,3355,3352,3350,3348,3345,3343,3340,3338,3335,3333,3331,3328,3326,3323,3321,3318,3316,3313,3311,3308,3306,3303,3301,3298,3296,3293,3291,3288,3286,3283,3281,3278,3276,3273,3271,3268,3266,3263,3261,3258,3256,3253,3251,3248,3246,3243,3241,3238,3235,3233,3230,3228,3225,3223,3220,3217,3215,3212,3210,3207,3205,3202,3199,3197,3194,3192,3189,3186,3184,3181,3178,3176,3173,3171,3168,3165,3163,3160,3157,3155,3152,3150,3147,3144,3142,3139,3136,3134,3131,3128,3126,3123,3120,3118,3115,3112,3110,3107,3104,3101,3099,3096,3093,3091,3088,3085,3083,3080,3077,3074,3072,3069,3066,3064,3061,3058,3055,3053,3050,3047,3044,3042,3039,3036,3033,3031,3028,3025,3022,3020,3017,3014,3011,3009,3006,3003,3000,2997,2995,2992,2989,2986,2983,2981,2978,2975,2972,2969,2967,2964,2961,2958,2955,2953,2950,2947,2944,2941,2938,2936,2933,2930,2927,2924,2921,2919,2916,2913,2910,2907,2904,2902,2899,2896,2893,2890,2887,2884,2882,2879,2876,2873,2870,2867,2864,2861,2859,2856,2853,2850,2847,2844,2841,2838,2835,2832,2830,2827,2824,2821,2818,2815,2812,2809,2806,2803,2800,2798,2795,2792,2789,2786,2783,2780,2777,2774,2771,2768,2765,2762,2759,2756,2754,2751,2748,2745,2742,2739,2736,2733,2730,2727,2724,2721,2718,2715,2712,2709,2706,2703,2700,2697,2694,2691,2688,2685,2682,2679,2676,2673,2670,2667,2664,2661,2658,2655,2652,2649,2646,2643,2640,2637,2634,2631,2628,2625,2622,2619,2616,2613,2610,2607,2604,2601,2598,2595,2592,2589,2586,2583,2580,2577,2574,2571,2568,2565,2562,2559,2556,2553,2550,2547,2543,2540,2537,2534,2531,2528,2525,2522,2519,2516,2513,2510,2507,2504,2501,2498,2495,2492,2488,2485,2482,2479,2476,2473,2470,2467,2464,2461,2458,2455,2452,2448,2445,2442,2439,2436,2433,2430,2427,2424,2421,2418,2415,2411,2408,2405,2402,2399,2396,2393,2390,2387,2384,2381,2377,2374,2371,2368,2365,2362,2359,2356,2353,2349,2346,2343,2340,2337,2334,2331,2328,2325,2321,2318,2315,2312,2309,2306,2303,2300,2297,2293,2290,2287,2284,2281,2278,2275,2272,2268,2265,2262,2259,2256,2253,2250,2247,2244,2240,2237,2234,2231,2228,2225,2222,2218,2215,2212,2209,2206,2203,2200,2197,2193,2190,2187,2184,2181,2178,2175,2171,2168,2165,2162,2159,2156,2153,2150,2146,2143,2140,2137,2134,2131,2128,2124,2121,2118,2115,2112,2109,2106,2102,2099,2096,2093,2090,2087,2084,2080,2077,2074,2071,2068,2065,2062,2058,2055,2052,2049,2046,2043,2040,2037,2033,2030,2027,2024,2021,2018,2015,2011,2008,2005,2002,1999,1996,1993,1989,1986,1983,1980,1977,1974,1971,1967,1964,1961,1958,1955,1952,1949,1945,1942,1939,1936,1933,1930,1927,1924,1920,1917,1914,1911,1908,1905,1902,1898,1895,1892,1889,1886,1883,1880,1877,1873,1870,1867,1864,1861,1858,1855,1851,1848,1845,1842,1839,1836,1833,1830,1827,1823,1820,1817,1814,1811,1808,1805,1802,1798,1795,1792,1789,1786,1783,1780,1777,1774,1770,1767,1764,1761,1758,1755,1752,1749,1746,1742,1739,1736,1733,1730,1727,1724,1721,1718,1714,1711,1708,1705,1702,1699,1696,1693,1690,1687,1684,1680,1677,1674,1671,1668,1665,1662,1659,1656,1653,1650,1647,1643,1640,1637,1634,1631,1628,1625,1622,1619,1616,1613,1610,1607,1603,1600,1597,1594,1591,1588,1585,1582,1579,1576,1573,1570,1567,1564,1561,1558,1555,1552,1548,1545,1542,1539,1536,1533,1530,1527,1524,1521,1518,1515,1512,1509,1506,1503,1500,1497,1494,1491,1488,1485,1482,1479,1476,1473,1470,1467,1464,1461,1458,1455,1452,1449,1446,1443,1440,1437,1434,1431,1428,1425,1422,1419,1416,1413,1410,1407,1404,1401,1398,1395,1392,1389,1386,1383,1380,1377,1374,1371,1368,1365,1362,1359,1356,1353,1350,1347,1344,1341,1339,1336,1333,1330,1327,1324,1321,1318,1315,1312,1309,1306,1303,1300,1297,1295,1292,1289,1286,1283,1280,1277,1274,1271,1268,1265,1263,1260,1257,1254,1251,1248,1245,1242,1239,1236,1234,1231,1228,1225,1222,1219,1216,1213,1211,1208,1205,1202,1199,1196,1193,1191,1188,1185,1182,1179,1176,1174,1171,1168,1165,1162,1159,1157,1154,1151,1148,1145,1142,1140,1137,1134,1131,1128,1126,1123,1120,1117,1114,1112,1109,1106,1103,1100,1098,1095,1092,1089,1086,1084,1081,1078,1075,1073,1070,1067,1064,1062,1059,1056,1053,1051,1048,1045,1042,1040,1037,1034,1031,1029,1026,1023,1021,1018,1015,1012,1010,1007,1004,1002,999,996,994,991,988,985,983,980,977,975,972,969,967,964,961,959,956,953,951,948,945,943,940,938,935,932,930,927,924,922,919,917,914,911,909,906,903,901,898,896,893,890,888,885,883,880,878,875,872,870,867,865,862,860,857,854,852,849,847,844,842,839,837,834,832,829,827,824,822,819,817,814,812,809,807,804,802,799,797,794,792,789,787,784,782,779,777,774,772,769,767,764,762,760,757,755,752,750,747,745,743,740,738,735,733,731,728,726,723,721,719,716,714,711,709,707,704,702,700,697,695,692,690,688,685,683,681,678,676,674,671,669,667,664,662,660,657,655,653,651,648,646,644,641,639,637,635,632,630,628,626,623,621,619,617,614,612,610,608,605,603,601,599,596,594,592,590,588,585,583,581,579,577,574,572,570,568,566,564,561,559,557,555,553,551,549,546,544,542,540,538,536,534,532,529,527,525,523,521,519,517,515,513,511,509,506,504,502,500,498,496,494,492,490,488,486,484,482,480,478,476,474,472,470,468,466,464,462,460,458,456,454,452,450,448,446,444,442,440,438,436,434,433,431,429,427,425,423,421,419,417,415,413,412,410,408,406,404,402,400,398,397,395,393,391,389,387,385,384,382,380,378,376,375,373,371,369,367,366,364,362,360,358,357,355,353,351,350,348,346,344,343,341,339,337,336,334,332,331,329,327,325,324,322,320,319,317,315,314,312,310,309,307,305,304,302,300,299,297,296,294,292,291,289,287,286,284,283,281,279,278,276,275,273,272,270,268,267,265,264,262,261,259,258,256,255,253,252,250,249,247,246,244,243,241,240,238,237,235,234,232,231,230,228,227,225,224,222,221,220,218,217,215,214,213,211,210,208,207,206,204,203,202,200,199,197,196,195,193,192,191,189,188,187,186,184,183,182,180,179,178,177,175,174,173,171,170,169,168,166,165,164,163,162,160,159,158,157,155,154,153,152,151,150,148,147,146,145,144,143,141,140,139,138,137,136,135,134,132,131,130,129,128,127,126,125,124,123,122,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,95,94,94,93,92,91,90,89,88,87,86,85,84,84,83,82,81,80,79,78,77,77,76,75,74,73,72,72,71,70,69,68,68,67,66,65,64,64,63,62,61,61,60,59,58,58,57,56,55,55,54,53,53,52,51,50,50,49,48,48,47,46,46,45,44,44,43,42,42,41,41,40,39,39,38,38,37,36,36,35,35,34,34,33,32,32,31,31,30,30,29,29,28,28,27,27,26,26,25,25,24,24,23,23,22,22,21,21,21,20,20,19,19,18,18,18,17,17,16,16,16,15,15,14,14,14,13,13,13,12,12,12,11,11,11,10,10,10,10,9,9,9,8,8,8,8,7,7,7,7,6,6,6,6,5,5,5,5,5,4,4,4,4,4,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,4,4,5,5,5,5,6,6,6,6,6,7,7,7,7,8,8,8,9,9,9,9,10,10,10,11,11,11,12,12,12,13,13,13,14,14,14,15,15,15,16,16,17,17,17,18,18,19,19,19,20,20,21,21,22,22,23,23,23,24,24,25,25,26,26,27,27,28,28,29,29,30,31,31,32,32,33,33,34,34,35,36,36,37,37,38,38,39,40,40,41,42,42,43,43,44,45,45,46,47,47,48,49,49,50,51,51,52,53,54,54,55,56,56,57,58,59,59,60,61,62,62,63,64,65,66,66,67,68,69,70,70,71,72,73,74,74,75,76,77,78,79,80,80,81,82,83,84,85,86,87,88,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,129,130,131,132,133,134,135,136,137,139,140,141,142,143,144,145,147,148,149,150,151,153,154,155,156,157,159,160,161,162,163,165,166,167,168,170,171,172,173,175,176,177,178,180,181,182,184,185,186,188,189,190,191,193,194,195,197,198,199,201,202,204,205,206,208,209,210,212,213,215,216,217,219,220,222,223,225,226,227,229,230,232,233,235,236,238,239,240,242,243,245,246,248,249,251,252,254,255,257,258,260,262,263,265,266,268,269,271,272,274,276,277,279,280,282,283,285,287,288,290,291,293,295,296,298,300,301,303,305,306,308,309,311,313,314,316,318,319,321,323,325,326,328,330,331,333,335,337,338,340,342,343,345,347,349,350,352,354,356,358,359,361,363,365,366,368,370,372,374,375,377,379,381,383,385,386,388,390,392,394,396,397,399,401,403,405,407,409,411,412,414,416,418,420,422,424,426,428,430,432,433,435,437,439,441,443,445,447,449,451,453,455,457,459,461,463,465,467,469,471,473,475,477,479,481,483,485,487,489,491,493,495,497,499,501,503,505,508,510,512,514,516,518,520,522,524,526,528,530,533,535,537,539,541,543,545,547,550,552,554,556,558,560,563,565,567,569,571,573,576,578,580,582,584,587,589,591,593,595,598,600,602,604,606,609,611,613,615,618,620,622,624,627,629,631,633,636,638,640,643,645,647,649,652,654,656,659,661,663,666,668,670,673,675,677,680,682,684,687,689,691,694,696,698,701,703,705,708,710,713,715,717,720,722,725,727,729,732,734,737,739,741,744,746,749,751,753,756,758,761,763,766,768,771,773,776,778,780,783,785,788,790,793,795,798,800,803,805,808,810,813,815,818,820,823,825,828,830,833,835,838,840,843,846,848,851,853,856,858,861,863,866,869,871,874,876,879,881,884,887,889,892,894,897,900,902,905,907,910,913,915,918,920,923,926,928,931,934,936,939,942,944,947,949,952,955,957,960,963,965,968,971,973,976,979,981,984,987,990,992,995,998,1000,1003,1006,1008,1011,1014,1017,1019,1022,1025,1027,1030,1033,1036,1038,1041,1044,1047,1049,1052,1055,1057,1060,1063,1066,1069,1071,1074,1077,1080,1082,1085,1088,1091,1093,1096,1099,1102,1105,1107,1110,1113,1116,1119,1121,1124,1127,1130,1133,1135,1138,1141,1144,1147,1149,1152,1155,1158,1161,1164,1166,1169,1172,1175,1178,1181,1183,1186,1189,1192,1195,1198,1201,1203,1206,1209,1212,1215,1218,1221,1224,1226,1229,1232,1235,1238,1241,1244,1247,1249,1252,1255,1258,1261,1264,1267,1270,1273,1276,1278,1281,1284,1287,1290,1293,1296,1299,1302,1305,1308,1311,1314,1316,1319,1322,1325,1328,1331,1334,1337,1340,1343,1346,1349,1352,1355,1358,1361,1364,1367,1370,1373,1375,1378,1381,1384,1387,1390,1393,1396,1399,1402,1405,1408,1411,1414,1417,1420,1423,1426,1429,1432,1435,1438,1441,1444,1447,1450,1453,1456,1459,1462,1465,1468,1471,1474,1477,1480,1483,1486,1489,1492,1495,1498,1501,1504,1507,1510,1514,1517,1520,1523,1526,1529,1532,1535,1538,1541,1544,1547,1550,1553,1556,1559,1562,1565,1568,1571,1574,1577,1581,1584,1587,1590,1593,1596,1599,1602,1605,1608,1611,1614,1617,1620,1623,1627,1630,1633,1636,1639,1642,1645,1648,1651,1654,1657,1660,1663,1667,1670,1673,1676,1679,1682,1685,1688,1691,1694,1697,1701,1704,1707,1710,1713,1716,1719,1722,1725,1728,1732,1735,1738,1741,1744,1747,1750,1753,1756,1760,1763,1766,1769,1772,1775,1778,1781,1784,1788,1791,1794,1797,1800,1803,1806,1809,1812,1816,1819,1822,1825,1828,1831,1834,1837,1841,1844,1847,1850,1853,1856,1859,1862,1866,1869,1872,1875,1878,1881,1884,1887,1891,1894,1897,1900,1903,1906,1909,1913,1916,1919,1922,1925,1928,1931,1934,1938,1941,1944,1947,1950,1953,1956,1960,1963,1966,1969,1972,1975,1978,1982,1985,1988,1991,1994,1997,2000,2004,2007,2010,2013,2016,2019,2022,2026,2029,2032,2035,2038,2041,2044,2047
};