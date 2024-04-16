/*!
 * Wavetable sine 12bit 8192
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

const uint16_t sine_12bit[8192] = {
2048,2049,2051,2052,2054,2055,2057,2058,2060,2062,2063,2065,2066,2068,2069,2071,2073,2074,2076,2077,2079,2080,2082,2084,2085,2087,2088,2090,2091,2093,2095,2096,2098,2099,2101,2102,2104,2106,2107,2109,2110,2112,2113,2115,2117,2118,2120,2121,2123,2124,2126,2128,2129,2131,2132,2134,2135,2137,2139,2140,2142,2143,2145,2146,2148,2150,2151,2153,2154,2156,2157,2159,2160,2162,2164,2165,2167,2168,2170,2171,2173,2175,2176,2178,2179,2181,2182,2184,2186,2187,2189,2190,2192,2193,2195,2197,2198,2200,2201,2203,2204,2206,2207,2209,2211,2212,2214,2215,2217,2218,2220,2222,2223,2225,2226,2228,2229,2231,2233,2234,2236,2237,2239,2240,2242,2243,2245,2247,2248,2250,2251,2253,2254,2256,2258,2259,2261,2262,2264,2265,2267,2268,2270,2272,2273,2275,2276,2278,2279,2281,2283,2284,2286,2287,2289,2290,2292,2293,2295,2297,2298,2300,2301,2303,2304,2306,2307,2309,2311,2312,2314,2315,2317,2318,2320,2321,2323,2325,2326,2328,2329,2331,2332,2334,2335,2337,2339,2340,2342,2343,2345,2346,2348,2349,2351,2353,2354,2356,2357,2359,2360,2362,2363,2365,2367,2368,2370,2371,2373,2374,2376,2377,2379,2380,2382,2384,2385,2387,2388,2390,2391,2393,2394,2396,2398,2399,2401,2402,2404,2405,2407,2408,2410,2411,2413,2415,2416,2418,2419,2421,2422,2424,2425,2427,2428,2430,2431,2433,2435,2436,2438,2439,2441,2442,2444,2445,2447,2448,2450,2452,2453,2455,2456,2458,2459,2461,2462,2464,2465,2467,2468,2470,2472,2473,2475,2476,2478,2479,2481,2482,2484,2485,2487,2488,2490,2491,2493,2495,2496,2498,2499,2501,2502,2504,2505,2507,2508,2510,2511,2513,2514,2516,2517,2519,2521,2522,2524,2525,2527,2528,2530,2531,2533,2534,2536,2537,2539,2540,2542,2543,2545,2546,2548,2550,2551,2553,2554,2556,2557,2559,2560,2562,2563,2565,2566,2568,2569,2571,2572,2574,2575,2577,2578,2580,2581,2583,2584,2586,2587,2589,2590,2592,2594,2595,2597,2598,2600,2601,2603,2604,2606,2607,2609,2610,2612,2613,2615,2616,2618,2619,2621,2622,2624,2625,2627,2628,2630,2631,2633,2634,2636,2637,2639,2640,2642,2643,2645,2646,2648,2649,2651,2652,2654,2655,2657,2658,2660,2661,2663,2664,2666,2667,2669,2670,2672,2673,2675,2676,2678,2679,2681,2682,2684,2685,2687,2688,2690,2691,2693,2694,2696,2697,2699,2700,2702,2703,2705,2706,2708,2709,2711,2712,2713,2715,2716,2718,2719,2721,2722,2724,2725,2727,2728,2730,2731,2733,2734,2736,2737,2739,2740,2742,2743,2745,2746,2748,2749,2750,2752,2753,2755,2756,2758,2759,2761,2762,2764,2765,2767,2768,2770,2771,2773,2774,2775,2777,2778,2780,2781,2783,2784,2786,2787,2789,2790,2792,2793,2795,2796,2797,2799,2800,2802,2803,2805,2806,2808,2809,2811,2812,2814,2815,2816,2818,2819,2821,2822,2824,2825,2827,2828,2829,2831,2832,2834,2835,2837,2838,2840,2841,2843,2844,2845,2847,2848,2850,2851,2853,2854,2856,2857,2858,2860,2861,2863,2864,2866,2867,2869,2870,2871,2873,2874,2876,2877,2879,2880,2881,2883,2884,2886,2887,2889,2890,2891,2893,2894,2896,2897,2899,2900,2901,2903,2904,2906,2907,2909,2910,2911,2913,2914,2916,2917,2919,2920,2921,2923,2924,2926,2927,2928,2930,2931,2933,2934,2936,2937,2938,2940,2941,2943,2944,2945,2947,2948,2950,2951,2952,2954,2955,2957,2958,2960,2961,2962,2964,2965,2967,2968,2969,2971,2972,2974,2975,2976,2978,2979,2981,2982,2983,2985,2986,2988,2989,2990,2992,2993,2995,2996,2997,2999,3000,3001,3003,3004,3006,3007,3008,3010,3011,3013,3014,3015,3017,3018,3019,3021,3022,3024,3025,3026,3028,3029,3031,3032,3033,3035,3036,3037,3039,3040,3042,3043,3044,3046,3047,3048,3050,3051,3052,3054,3055,3057,3058,3059,3061,3062,3063,3065,3066,3067,3069,3070,3072,3073,3074,3076,3077,3078,3080,3081,3082,3084,3085,3086,3088,3089,3091,3092,3093,3095,3096,3097,3099,3100,3101,3103,3104,3105,3107,3108,3109,3111,3112,3113,3115,3116,3117,3119,3120,3121,3123,3124,3125,3127,3128,3129,3131,3132,3133,3135,3136,3137,3139,3140,3141,3143,3144,3145,3147,3148,3149,3151,3152,3153,3155,3156,3157,3159,3160,3161,3163,3164,3165,3167,3168,3169,3170,3172,3173,3174,3176,3177,3178,3180,3181,3182,3184,3185,3186,3187,3189,3190,3191,3193,3194,3195,3197,3198,3199,3200,3202,3203,3204,3206,3207,3208,3210,3211,3212,3213,3215,3216,3217,3219,3220,3221,3222,3224,3225,3226,3228,3229,3230,3231,3233,3234,3235,3237,3238,3239,3240,3242,3243,3244,3245,3247,3248,3249,3251,3252,3253,3254,3256,3257,3258,3259,3261,3262,3263,3265,3266,3267,3268,3270,3271,3272,3273,3275,3276,3277,3278,3280,3281,3282,3283,3285,3286,3287,3288,3290,3291,3292,3293,3295,3296,3297,3298,3300,3301,3302,3303,3305,3306,3307,3308,3309,3311,3312,3313,3314,3316,3317,3318,3319,3321,3322,3323,3324,3325,3327,3328,3329,3330,3332,3333,3334,3335,3337,3338,3339,3340,3341,3343,3344,3345,3346,3347,3349,3350,3351,3352,3354,3355,3356,3357,3358,3360,3361,3362,3363,3364,3366,3367,3368,3369,3370,3372,3373,3374,3375,3376,3378,3379,3380,3381,3382,3383,3385,3386,3387,3388,3389,3391,3392,3393,3394,3395,3397,3398,3399,3400,3401,3402,3404,3405,3406,3407,3408,3409,3411,3412,3413,3414,3415,3416,3418,3419,3420,3421,3422,3423,3425,3426,3427,3428,3429,3430,3432,3433,3434,3435,3436,3437,3439,3440,3441,3442,3443,3444,3445,3447,3448,3449,3450,3451,3452,3453,3455,3456,3457,3458,3459,3460,3461,3463,3464,3465,3466,3467,3468,3469,3470,3472,3473,3474,3475,3476,3477,3478,3479,3481,3482,3483,3484,3485,3486,3487,3488,3490,3491,3492,3493,3494,3495,3496,3497,3498,3500,3501,3502,3503,3504,3505,3506,3507,3508,3509,3511,3512,3513,3514,3515,3516,3517,3518,3519,3520,3521,3523,3524,3525,3526,3527,3528,3529,3530,3531,3532,3533,3534,3536,3537,3538,3539,3540,3541,3542,3543,3544,3545,3546,3547,3548,3550,3551,3552,3553,3554,3555,3556,3557,3558,3559,3560,3561,3562,3563,3564,3565,3566,3568,3569,3570,3571,3572,3573,3574,3575,3576,3577,3578,3579,3580,3581,3582,3583,3584,3585,3586,3587,3588,3589,3590,3591,3593,3594,3595,3596,3597,3598,3599,3600,3601,3602,3603,3604,3605,3606,3607,3608,3609,3610,3611,3612,3613,3614,3615,3616,3617,3618,3619,3620,3621,3622,3623,3624,3625,3626,3627,3628,3629,3630,3631,3632,3633,3634,3635,3636,3637,3638,3639,3640,3641,3642,3643,3644,3645,3646,3647,3648,3649,3650,3651,3652,3653,3654,3655,3656,3657,3658,3658,3659,3660,3661,3662,3663,3664,3665,3666,3667,3668,3669,3670,3671,3672,3673,3674,3675,3676,3677,3678,3679,3680,3681,3681,3682,3683,3684,3685,3686,3687,3688,3689,3690,3691,3692,3693,3694,3695,3696,3696,3697,3698,3699,3700,3701,3702,3703,3704,3705,3706,3707,3708,3708,3709,3710,3711,3712,3713,3714,3715,3716,3717,3718,3719,3719,3720,3721,3722,3723,3724,3725,3726,3727,3728,3728,3729,3730,3731,3732,3733,3734,3735,3736,3736,3737,3738,3739,3740,3741,3742,3743,3744,3744,3745,3746,3747,3748,3749,3750,3751,3751,3752,3753,3754,3755,3756,3757,3757,3758,3759,3760,3761,3762,3763,3763,3764,3765,3766,3767,3768,3769,3769,3770,3771,3772,3773,3774,3775,3775,3776,3777,3778,3779,3780,3780,3781,3782,3783,3784,3785,3785,3786,3787,3788,3789,3790,3790,3791,3792,3793,3794,3794,3795,3796,3797,3798,3799,3799,3800,3801,3802,3803,3803,3804,3805,3806,3807,3807,3808,3809,3810,3811,3811,3812,3813,3814,3815,3815,3816,3817,3818,3819,3819,3820,3821,3822,3822,3823,3824,3825,3826,3826,3827,3828,3829,3829,3830,3831,3832,3833,3833,3834,3835,3836,3836,3837,3838,3839,3839,3840,3841,3842,3842,3843,3844,3845,3845,3846,3847,3848,3848,3849,3850,3851,3851,3852,3853,3854,3854,3855,3856,3857,3857,3858,3859,3860,3860,3861,3862,3862,3863,3864,3865,3865,3866,3867,3868,3868,3869,3870,3870,3871,3872,3873,3873,3874,3875,3875,3876,3877,3877,3878,3879,3880,3880,3881,3882,3882,3883,3884,3884,3885,3886,3887,3887,3888,3889,3889,3890,3891,3891,3892,3893,3893,3894,3895,3895,3896,3897,3897,3898,3899,3899,3900,3901,3901,3902,3903,3903,3904,3905,3905,3906,3907,3907,3908,3909,3909,3910,3911,3911,3912,3913,3913,3914,3915,3915,3916,3916,3917,3918,3918,3919,3920,3920,3921,3922,3922,3923,3923,3924,3925,3925,3926,3927,3927,3928,3928,3929,3930,3930,3931,3932,3932,3933,3933,3934,3935,3935,3936,3936,3937,3938,3938,3939,3939,3940,3941,3941,3942,3942,3943,3944,3944,3945,3945,3946,3947,3947,3948,3948,3949,3949,3950,3951,3951,3952,3952,3953,3953,3954,3955,3955,3956,3956,3957,3957,3958,3959,3959,3960,3960,3961,3961,3962,3962,3963,3964,3964,3965,3965,3966,3966,3967,3967,3968,3969,3969,3970,3970,3971,3971,3972,3972,3973,3973,3974,3974,3975,3975,3976,3977,3977,3978,3978,3979,3979,3980,3980,3981,3981,3982,3982,3983,3983,3984,3984,3985,3985,3986,3986,3987,3987,3988,3988,3989,3989,3990,3990,3991,3991,3992,3992,3993,3993,3994,3994,3995,3995,3996,3996,3997,3997,3998,3998,3999,3999,3999,4000,4000,4001,4001,4002,4002,4003,4003,4004,4004,4005,4005,4006,4006,4006,4007,4007,4008,4008,4009,4009,4010,4010,4011,4011,4011,4012,4012,4013,4013,4014,4014,4014,4015,4015,4016,4016,4017,4017,4017,4018,4018,4019,4019,4020,4020,4020,4021,4021,4022,4022,4023,4023,4023,4024,4024,4025,4025,4025,4026,4026,4027,4027,4027,4028,4028,4029,4029,4029,4030,4030,4031,4031,4031,4032,4032,4032,4033,4033,4034,4034,4034,4035,4035,4036,4036,4036,4037,4037,4037,4038,4038,4038,4039,4039,4040,4040,4040,4041,4041,4041,4042,4042,4042,4043,4043,4043,4044,4044,4044,4045,4045,4046,4046,4046,4047,4047,4047,4048,4048,4048,4049,4049,4049,4050,4050,4050,4051,4051,4051,4051,4052,4052,4052,4053,4053,4053,4054,4054,4054,4055,4055,4055,4056,4056,4056,4056,4057,4057,4057,4058,4058,4058,4059,4059,4059,4059,4060,4060,4060,4061,4061,4061,4061,4062,4062,4062,4063,4063,4063,4063,4064,4064,4064,4064,4065,4065,4065,4065,4066,4066,4066,4067,4067,4067,4067,4068,4068,4068,4068,4069,4069,4069,4069,4070,4070,4070,4070,4071,4071,4071,4071,4071,4072,4072,4072,4072,4073,4073,4073,4073,4074,4074,4074,4074,4074,4075,4075,4075,4075,4076,4076,4076,4076,4076,4077,4077,4077,4077,4077,4078,4078,4078,4078,4078,4079,4079,4079,4079,4079,4080,4080,4080,4080,4080,4080,4081,4081,4081,4081,4081,4082,4082,4082,4082,4082,4082,4083,4083,4083,4083,4083,4083,4084,4084,4084,4084,4084,4084,4085,4085,4085,4085,4085,4085,4085,4086,4086,4086,4086,4086,4086,4086,4087,4087,4087,4087,4087,4087,4087,4088,4088,4088,4088,4088,4088,4088,4088,4089,4089,4089,4089,4089,4089,4089,4089,4089,4090,4090,4090,4090,4090,4090,4090,4090,4090,4090,4091,4091,4091,4091,4091,4091,4091,4091,4091,4091,4091,4092,4092,4092,4092,4092,4092,4092,4092,4092,4092,4092,4092,4092,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4093,4092,4092,4092,4092,4092,4092,4092,4092,4092,4092,4092,4092,4092,4091,4091,4091,4091,4091,4091,4091,4091,4091,4091,4091,4090,4090,4090,4090,4090,4090,4090,4090,4090,4089,4089,4089,4089,4089,4089,4089,4089,4089,4088,4088,4088,4088,4088,4088,4088,4088,4087,4087,4087,4087,4087,4087,4087,4087,4086,4086,4086,4086,4086,4086,4086,4085,4085,4085,4085,4085,4085,4084,4084,4084,4084,4084,4084,4083,4083,4083,4083,4083,4083,4082,4082,4082,4082,4082,4082,4081,4081,4081,4081,4081,4081,4080,4080,4080,4080,4080,4079,4079,4079,4079,4079,4078,4078,4078,4078,4078,4077,4077,4077,4077,4077,4076,4076,4076,4076,4076,4075,4075,4075,4075,4075,4074,4074,4074,4074,4073,4073,4073,4073,4073,4072,4072,4072,4072,4071,4071,4071,4071,4070,4070,4070,4070,4069,4069,4069,4069,4068,4068,4068,4068,4067,4067,4067,4067,4066,4066,4066,4066,4065,4065,4065,4065,4064,4064,4064,4063,4063,4063,4063,4062,4062,4062,4062,4061,4061,4061,4060,4060,4060,4060,4059,4059,4059,4058,4058,4058,4057,4057,4057,4057,4056,4056,4056,4055,4055,4055,4054,4054,4054,4054,4053,4053,4053,4052,4052,4052,4051,4051,4051,4050,4050,4050,4049,4049,4049,4048,4048,4048,4047,4047,4047,4046,4046,4046,4045,4045,4045,4044,4044,4044,4043,4043,4043,4042,4042,4042,4041,4041,4040,4040,4040,4039,4039,4039,4038,4038,4038,4037,4037,4036,4036,4036,4035,4035,4035,4034,4034,4033,4033,4033,4032,4032,4032,4031,4031,4030,4030,4030,4029,4029,4028,4028,4028,4027,4027,4026,4026,4026,4025,4025,4024,4024,4024,4023,4023,4022,4022,4021,4021,4021,4020,4020,4019,4019,4019,4018,4018,4017,4017,4016,4016,4016,4015,4015,4014,4014,4013,4013,4012,4012,4012,4011,4011,4010,4010,4009,4009,4008,4008,4008,4007,4007,4006,4006,4005,4005,4004,4004,4003,4003,4003,4002,4002,4001,4001,4000,4000,3999,3999,3998,3998,3997,3997,3996,3996,3995,3995,3994,3994,3993,3993,3993,3992,3992,3991,3991,3990,3990,3989,3989,3988,3988,3987,3987,3986,3986,3985,3985,3984,3984,3983,3982,3982,3981,3981,3980,3980,3979,3979,3978,3978,3977,3977,3976,3976,3975,3975,3974,3974,3973,3973,3972,3971,3971,3970,3970,3969,3969,3968,3968,3967,3967,3966,3966,3965,3964,3964,3963,3963,3962,3962,3961,3961,3960,3959,3959,3958,3958,3957,3957,3956,3955,3955,3954,3954,3953,3953,3952,3951,3951,3950,3950,3949,3949,3948,3947,3947,3946,3946,3945,3944,3944,3943,3943,3942,3941,3941,3940,3940,3939,3938,3938,3937,3937,3936,3935,3935,3934,3934,3933,3932,3932,3931,3931,3930,3929,3929,3928,3927,3927,3926,3926,3925,3924,3924,3923,3922,3922,3921,3921,3920,3919,3919,3918,3917,3917,3916,3915,3915,3914,3914,3913,3912,3912,3911,3910,3910,3909,3908,3908,3907,3906,3906,3905,3904,3904,3903,3902,3902,3901,3900,3900,3899,3898,3898,3897,3896,3896,3895,3894,3894,3893,3892,3892,3891,3890,3890,3889,3888,3888,3887,3886,3885,3885,3884,3883,3883,3882,3881,3881,3880,3879,3879,3878,3877,3876,3876,3875,3874,3874,3873,3872,3871,3871,3870,3869,3869,3868,3867,3866,3866,3865,3864,3864,3863,3862,3861,3861,3860,3859,3858,3858,3857,3856,3856,3855,3854,3853,3853,3852,3851,3850,3850,3849,3848,3847,3847,3846,3845,3844,3844,3843,3842,3841,3841,3840,3839,3838,3838,3837,3836,3835,3834,3834,3833,3832,3831,3831,3830,3829,3828,3828,3827,3826,3825,3824,3824,3823,3822,3821,3821,3820,3819,3818,3817,3817,3816,3815,3814,3813,3813,3812,3811,3810,3809,3809,3808,3807,3806,3805,3805,3804,3803,3802,3801,3801,3800,3799,3798,3797,3797,3796,3795,3794,3793,3792,3792,3791,3790,3789,3788,3787,3787,3786,3785,3784,3783,3782,3782,3781,3780,3779,3778,3777,3777,3776,3775,3774,3773,3772,3772,3771,3770,3769,3768,3767,3766,3766,3765,3764,3763,3762,3761,3760,3760,3759,3758,3757,3756,3755,3754,3754,3753,3752,3751,3750,3749,3748,3747,3747,3746,3745,3744,3743,3742,3741,3740,3740,3739,3738,3737,3736,3735,3734,3733,3732,3732,3731,3730,3729,3728,3727,3726,3725,3724,3723,3723,3722,3721,3720,3719,3718,3717,3716,3715,3714,3714,3713,3712,3711,3710,3709,3708,3707,3706,3705,3704,3703,3702,3702,3701,3700,3699,3698,3697,3696,3695,3694,3693,3692,3691,3690,3689,3689,3688,3687,3686,3685,3684,3683,3682,3681,3680,3679,3678,3677,3676,3675,3674,3673,3672,3671,3671,3670,3669,3668,3667,3666,3665,3664,3663,3662,3661,3660,3659,3658,3657,3656,3655,3654,3653,3652,3651,3650,3649,3648,3647,3646,3645,3644,3643,3642,3641,3640,3639,3638,3637,3636,3635,3634,3633,3632,3631,3630,3629,3628,3627,3627,3626,3625,3623,3622,3621,3620,3619,3618,3617,3616,3615,3614,3613,3612,3611,3610,3609,3608,3607,3606,3605,3604,3603,3602,3601,3600,3599,3598,3597,3596,3595,3594,3593,3592,3591,3590,3589,3588,3587,3586,3585,3584,3583,3582,3581,3580,3579,3577,3576,3575,3574,3573,3572,3571,3570,3569,3568,3567,3566,3565,3564,3563,3562,3561,3560,3559,3557,3556,3555,3554,3553,3552,3551,3550,3549,3548,3547,3546,3545,3544,3543,3541,3540,3539,3538,3537,3536,3535,3534,3533,3532,3531,3530,3529,3527,3526,3525,3524,3523,3522,3521,3520,3519,3518,3517,3515,3514,3513,3512,3511,3510,3509,3508,3507,3506,3504,3503,3502,3501,3500,3499,3498,3497,3496,3495,3493,3492,3491,3490,3489,3488,3487,3486,3485,3483,3482,3481,3480,3479,3478,3477,3476,3474,3473,3472,3471,3470,3469,3468,3467,3465,3464,3463,3462,3461,3460,3459,3457,3456,3455,3454,3453,3452,3451,3449,3448,3447,3446,3445,3444,3443,3441,3440,3439,3438,3437,3436,3434,3433,3432,3431,3430,3429,3428,3426,3425,3424,3423,3422,3421,3419,3418,3417,3416,3415,3414,3412,3411,3410,3409,3408,3407,3405,3404,3403,3402,3401,3399,3398,3397,3396,3395,3394,3392,3391,3390,3389,3388,3386,3385,3384,3383,3382,3381,3379,3378,3377,3376,3375,3373,3372,3371,3370,3369,3367,3366,3365,3364,3363,3361,3360,3359,3358,3357,3355,3354,3353,3352,3350,3349,3348,3347,3346,3344,3343,3342,3341,3340,3338,3337,3336,3335,3333,3332,3331,3330,3329,3327,3326,3325,3324,3322,3321,3320,3319,3318,3316,3315,3314,3313,3311,3310,3309,3308,3306,3305,3304,3303,3301,3300,3299,3298,3296,3295,3294,3293,3291,3290,3289,3288,3286,3285,3284,3283,3281,3280,3279,3278,3276,3275,3274,3273,3271,3270,3269,3268,3266,3265,3264,3263,3261,3260,3259,3258,3256,3255,3254,3252,3251,3250,3249,3247,3246,3245,3244,3242,3241,3240,3238,3237,3236,3235,3233,3232,3231,3230,3228,3227,3226,3224,3223,3222,3221,3219,3218,3217,3215,3214,3213,3211,3210,3209,3208,3206,3205,3204,3202,3201,3200,3199,3197,3196,3195,3193,3192,3191,3189,3188,3187,3186,3184,3183,3182,3180,3179,3178,3176,3175,3174,3172,3171,3170,3169,3167,3166,3165,3163,3162,3161,3159,3158,3157,3155,3154,3153,3151,3150,3149,3147,3146,3145,3143,3142,3141,3139,3138,3137,3135,3134,3133,3131,3130,3129,3127,3126,3125,3123,3122,3121,3119,3118,3117,3115,3114,3113,3111,3110,3109,3107,3106,3105,3103,3102,3101,3099,3098,3097,3095,3094,3093,3091,3090,3089,3087,3086,3084,3083,3082,3080,3079,3078,3076,3075,3074,3072,3071,3070,3068,3067,3065,3064,3063,3061,3060,3059,3057,3056,3055,3053,3052,3050,3049,3048,3046,3045,3044,3042,3041,3039,3038,3037,3035,3034,3033,3031,3030,3028,3027,3026,3024,3023,3022,3020,3019,3017,3016,3015,3013,3012,3010,3009,3008,3006,3005,3004,3002,3001,2999,2998,2997,2995,2994,2992,2991,2990,2988,2987,2985,2984,2983,2981,2980,2978,2977,2976,2974,2973,2971,2970,2969,2967,2966,2964,2963,2962,2960,2959,2957,2956,2955,2953,2952,2950,2949,2948,2946,2945,2943,2942,2941,2939,2938,2936,2935,2933,2932,2931,2929,2928,2926,2925,2924,2922,2921,2919,2918,2916,2915,2914,2912,2911,2909,2908,2906,2905,2904,2902,2901,2899,2898,2896,2895,2894,2892,2891,2889,2888,2886,2885,2884,2882,2881,2879,2878,2876,2875,2874,2872,2871,2869,2868,2866,2865,2863,2862,2861,2859,2858,2856,2855,2853,2852,2850,2849,2848,2846,2845,2843,2842,2840,2839,2837,2836,2835,2833,2832,2830,2829,2827,2826,2824,2823,2822,2820,2819,2817,2816,2814,2813,2811,2810,2808,2807,2805,2804,2803,2801,2800,2798,2797,2795,2794,2792,2791,2789,2788,2786,2785,2784,2782,2781,2779,2778,2776,2775,2773,2772,2770,2769,2767,2766,2764,2763,2762,2760,2759,2757,2756,2754,2753,2751,2750,2748,2747,2745,2744,2742,2741,2739,2738,2736,2735,2733,2732,2731,2729,2728,2726,2725,2723,2722,2720,2719,2717,2716,2714,2713,2711,2710,2708,2707,2705,2704,2702,2701,2699,2698,2696,2695,2693,2692,2690,2689,2687,2686,2684,2683,2681,2680,2678,2677,2676,2674,2673,2671,2670,2668,2667,2665,2664,2662,2661,2659,2658,2656,2655,2653,2652,2650,2649,2647,2646,2644,2643,2641,2640,2638,2637,2635,2634,2632,2631,2629,2627,2626,2624,2623,2621,2620,2618,2617,2615,2614,2612,2611,2609,2608,2606,2605,2603,2602,2600,2599,2597,2596,2594,2593,2591,2590,2588,2587,2585,2584,2582,2581,2579,2578,2576,2575,2573,2572,2570,2569,2567,2565,2564,2562,2561,2559,2558,2556,2555,2553,2552,2550,2549,2547,2546,2544,2543,2541,2540,2538,2537,2535,2534,2532,2530,2529,2527,2526,2524,2523,2521,2520,2518,2517,2515,2514,2512,2511,2509,2508,2506,2504,2503,2501,2500,2498,2497,2495,2494,2492,2491,2489,2488,2486,2485,2483,2481,2480,2478,2477,2475,2474,2472,2471,2469,2468,2466,2465,2463,2462,2460,2458,2457,2455,2454,2452,2451,2449,2448,2446,2445,2443,2442,2440,2438,2437,2435,2434,2432,2431,2429,2428,2426,2425,2423,2421,2420,2418,2417,2415,2414,2412,2411,2409,2408,2406,2404,2403,2401,2400,2398,2397,2395,2394,2392,2391,2389,2387,2386,2384,2383,2381,2380,2378,2377,2375,2373,2372,2370,2369,2367,2366,2364,2363,2361,2360,2358,2356,2355,2353,2352,2350,2349,2347,2346,2344,2342,2341,2339,2338,2336,2335,2333,2332,2330,2328,2327,2325,2324,2322,2321,2319,2318,2316,2314,2313,2311,2310,2308,2307,2305,2304,2302,2300,2299,2297,2296,2294,2293,2291,2290,2288,2286,2285,2283,2282,2280,2279,2277,2275,2274,2272,2271,2269,2268,2266,2265,2263,2261,2260,2258,2257,2255,2254,2252,2251,2249,2247,2246,2244,2243,2241,2240,2238,2236,2235,2233,2232,2230,2229,2227,2225,2224,2222,2221,2219,2218,2216,2215,2213,2211,2210,2208,2207,2205,2204,2202,2200,2199,2197,2196,2194,2193,2191,2189,2188,2186,2185,2183,2182,2180,2179,2177,2175,2174,2172,2171,2169,2168,2166,2164,2163,2161,2160,2158,2157,2155,2153,2152,2150,2149,2147,2146,2144,2142,2141,2139,2138,2136,2135,2133,2131,2130,2128,2127,2125,2124,2122,2120,2119,2117,2116,2114,2113,2111,2110,2108,2106,2105,2103,2102,2100,2099,2097,2095,2094,2092,2091,2089,2088,2086,2084,2083,2081,2080,2078,2077,2075,2073,2072,2070,2069,2067,2066,2064,2062,2061,2059,2058,2056,2055,2053,2051,2050,2048,2047,2045,2044,2042,2040,2039,2037,2036,2034,2033,2031,2029,2028,2026,2025,2023,2022,2020,2018,2017,2015,2014,2012,2011,2009,2007,2006,2004,2003,2001,2000,1998,1996,1995,1993,1992,1990,1989,1987,1985,1984,1982,1981,1979,1978,1976,1975,1973,1971,1970,1968,1967,1965,1964,1962,1960,1959,1957,1956,1954,1953,1951,1949,1948,1946,1945,1943,1942,1940,1938,1937,1935,1934,1932,1931,1929,1927,1926,1924,1923,1921,1920,1918,1916,1915,1913,1912,1910,1909,1907,1906,1904,1902,1901,1899,1898,1896,1895,1893,1891,1890,1888,1887,1885,1884,1882,1880,1879,1877,1876,1874,1873,1871,1870,1868,1866,1865,1863,1862,1860,1859,1857,1855,1854,1852,1851,1849,1848,1846,1844,1843,1841,1840,1838,1837,1835,1834,1832,1830,1829,1827,1826,1824,1823,1821,1820,1818,1816,1815,1813,1812,1810,1809,1807,1805,1804,1802,1801,1799,1798,1796,1795,1793,1791,1790,1788,1787,1785,1784,1782,1781,1779,1777,1776,1774,1773,1771,1770,1768,1767,1765,1763,1762,1760,1759,1757,1756,1754,1753,1751,1749,1748,1746,1745,1743,1742,1740,1739,1737,1735,1734,1732,1731,1729,1728,1726,1725,1723,1722,1720,1718,1717,1715,1714,1712,1711,1709,1708,1706,1704,1703,1701,1700,1698,1697,1695,1694,1692,1691,1689,1687,1686,1684,1683,1681,1680,1678,1677,1675,1674,1672,1670,1669,1667,1666,1664,1663,1661,1660,1658,1657,1655,1653,1652,1650,1649,1647,1646,1644,1643,1641,1640,1638,1637,1635,1633,1632,1630,1629,1627,1626,1624,1623,1621,1620,1618,1617,1615,1614,1612,1610,1609,1607,1606,1604,1603,1601,1600,1598,1597,1595,1594,1592,1591,1589,1587,1586,1584,1583,1581,1580,1578,1577,1575,1574,1572,1571,1569,1568,1566,1565,1563,1561,1560,1558,1557,1555,1554,1552,1551,1549,1548,1546,1545,1543,1542,1540,1539,1537,1536,1534,1533,1531,1530,1528,1526,1525,1523,1522,1520,1519,1517,1516,1514,1513,1511,1510,1508,1507,1505,1504,1502,1501,1499,1498,1496,1495,1493,1492,1490,1489,1487,1486,1484,1483,1481,1480,1478,1477,1475,1474,1472,1471,1469,1468,1466,1464,1463,1461,1460,1458,1457,1455,1454,1452,1451,1449,1448,1446,1445,1443,1442,1440,1439,1437,1436,1434,1433,1431,1430,1428,1427,1425,1424,1422,1421,1419,1418,1417,1415,1414,1412,1411,1409,1408,1406,1405,1403,1402,1400,1399,1397,1396,1394,1393,1391,1390,1388,1387,1385,1384,1382,1381,1379,1378,1376,1375,1373,1372,1370,1369,1367,1366,1364,1363,1362,1360,1359,1357,1356,1354,1353,1351,1350,1348,1347,1345,1344,1342,1341,1339,1338,1336,1335,1333,1332,1331,1329,1328,1326,1325,1323,1322,1320,1319,1317,1316,1314,1313,1311,1310,1309,1307,1306,1304,1303,1301,1300,1298,1297,1295,1294,1292,1291,1290,1288,1287,1285,1284,1282,1281,1279,1278,1276,1275,1273,1272,1271,1269,1268,1266,1265,1263,1262,1260,1259,1258,1256,1255,1253,1252,1250,1249,1247,1246,1245,1243,1242,1240,1239,1237,1236,1234,1233,1232,1230,1229,1227,1226,1224,1223,1221,1220,1219,1217,1216,1214,1213,1211,1210,1209,1207,1206,1204,1203,1201,1200,1199,1197,1196,1194,1193,1191,1190,1189,1187,1186,1184,1183,1181,1180,1179,1177,1176,1174,1173,1171,1170,1169,1167,1166,1164,1163,1162,1160,1159,1157,1156,1154,1153,1152,1150,1149,1147,1146,1145,1143,1142,1140,1139,1138,1136,1135,1133,1132,1131,1129,1128,1126,1125,1124,1122,1121,1119,1118,1117,1115,1114,1112,1111,1110,1108,1107,1105,1104,1103,1101,1100,1098,1097,1096,1094,1093,1091,1090,1089,1087,1086,1085,1083,1082,1080,1079,1078,1076,1075,1073,1072,1071,1069,1068,1067,1065,1064,1062,1061,1060,1058,1057,1056,1054,1053,1051,1050,1049,1047,1046,1045,1043,1042,1040,1039,1038,1036,1035,1034,1032,1031,1030,1028,1027,1025,1024,1023,1021,1020,1019,1017,1016,1015,1013,1012,1011,1009,1008,1006,1005,1004,1002,1001,1000,998,997,996,994,993,992,990,989,988,986,985,984,982,981,980,978,977,976,974,973,972,970,969,968,966,965,964,962,961,960,958,957,956,954,953,952,950,949,948,946,945,944,942,941,940,938,937,936,934,933,932,930,929,928,926,925,924,923,921,920,919,917,916,915,913,912,911,909,908,907,906,904,903,902,900,899,898,896,895,894,893,891,890,889,887,886,885,884,882,881,880,878,877,876,874,873,872,871,869,868,867,865,864,863,862,860,859,858,857,855,854,853,851,850,849,848,846,845,844,843,841,840,839,837,836,835,834,832,831,830,829,827,826,825,824,822,821,820,819,817,816,815,814,812,811,810,809,807,806,805,804,802,801,800,799,797,796,795,794,792,791,790,789,787,786,785,784,782,781,780,779,777,776,775,774,773,771,770,769,768,766,765,764,763,762,760,759,758,757,755,754,753,752,751,749,748,747,746,745,743,742,741,740,738,737,736,735,734,732,731,730,729,728,726,725,724,723,722,720,719,718,717,716,714,713,712,711,710,709,707,706,705,704,703,701,700,699,698,697,696,694,693,692,691,690,688,687,686,685,684,683,681,680,679,678,677,676,674,673,672,671,670,669,667,666,665,664,663,662,661,659,658,657,656,655,654,652,651,650,649,648,647,646,644,643,642,641,640,639,638,636,635,634,633,632,631,630,628,627,626,625,624,623,622,621,619,618,617,616,615,614,613,612,610,609,608,607,606,605,604,603,602,600,599,598,597,596,595,594,593,592,591,589,588,587,586,585,584,583,582,581,580,578,577,576,575,574,573,572,571,570,569,568,566,565,564,563,562,561,560,559,558,557,556,555,554,552,551,550,549,548,547,546,545,544,543,542,541,540,539,538,536,535,534,533,532,531,530,529,528,527,526,525,524,523,522,521,520,519,518,516,515,514,513,512,511,510,509,508,507,506,505,504,503,502,501,500,499,498,497,496,495,494,493,492,491,490,489,488,487,486,485,484,483,482,481,480,479,478,477,476,475,474,473,472,470,469,468,468,467,466,465,464,463,462,461,460,459,458,457,456,455,454,453,452,451,450,449,448,447,446,445,444,443,442,441,440,439,438,437,436,435,434,433,432,431,430,429,428,427,426,425,424,424,423,422,421,420,419,418,417,416,415,414,413,412,411,410,409,408,407,406,406,405,404,403,402,401,400,399,398,397,396,395,394,393,393,392,391,390,389,388,387,386,385,384,383,382,381,381,380,379,378,377,376,375,374,373,372,372,371,370,369,368,367,366,365,364,363,363,362,361,360,359,358,357,356,355,355,354,353,352,351,350,349,348,348,347,346,345,344,343,342,341,341,340,339,338,337,336,335,335,334,333,332,331,330,329,329,328,327,326,325,324,323,323,322,321,320,319,318,318,317,316,315,314,313,313,312,311,310,309,308,308,307,306,305,304,303,303,302,301,300,299,298,298,297,296,295,294,294,293,292,291,290,290,289,288,287,286,286,285,284,283,282,282,281,280,279,278,278,277,276,275,274,274,273,272,271,271,270,269,268,267,267,266,265,264,264,263,262,261,261,260,259,258,257,257,256,255,254,254,253,252,251,251,250,249,248,248,247,246,245,245,244,243,242,242,241,240,239,239,238,237,237,236,235,234,234,233,232,231,231,230,229,229,228,227,226,226,225,224,224,223,222,221,221,220,219,219,218,217,216,216,215,214,214,213,212,212,211,210,210,209,208,207,207,206,205,205,204,203,203,202,201,201,200,199,199,198,197,197,196,195,195,194,193,193,192,191,191,190,189,189,188,187,187,186,185,185,184,183,183,182,181,181,180,180,179,178,178,177,176,176,175,174,174,173,173,172,171,171,170,169,169,168,168,167,166,166,165,164,164,163,163,162,161,161,160,160,159,158,158,157,157,156,155,155,154,154,153,152,152,151,151,150,149,149,148,148,147,146,146,145,145,144,144,143,142,142,141,141,140,140,139,138,138,137,137,136,136,135,134,134,133,133,132,132,131,131,130,129,129,128,128,127,127,126,126,125,125,124,124,123,122,122,121,121,120,120,119,119,118,118,117,117,116,116,115,115,114,114,113,113,112,111,111,110,110,109,109,108,108,107,107,106,106,105,105,104,104,103,103,102,102,102,101,101,100,100,99,99,98,98,97,97,96,96,95,95,94,94,93,93,92,92,92,91,91,90,90,89,89,88,88,87,87,87,86,86,85,85,84,84,83,83,83,82,82,81,81,80,80,79,79,79,78,78,77,77,76,76,76,75,75,74,74,74,73,73,72,72,71,71,71,70,70,69,69,69,68,68,67,67,67,66,66,65,65,65,64,64,63,63,63,62,62,62,61,61,60,60,60,59,59,59,58,58,57,57,57,56,56,56,55,55,55,54,54,53,53,53,52,52,52,51,51,51,50,50,50,49,49,49,48,48,48,47,47,47,46,46,46,45,45,45,44,44,44,43,43,43,42,42,42,41,41,41,41,40,40,40,39,39,39,38,38,38,38,37,37,37,36,36,36,35,35,35,35,34,34,34,33,33,33,33,32,32,32,32,31,31,31,30,30,30,30,29,29,29,29,28,28,28,28,27,27,27,27,26,26,26,26,25,25,25,25,24,24,24,24,23,23,23,23,22,22,22,22,22,21,21,21,21,20,20,20,20,20,19,19,19,19,19,18,18,18,18,18,17,17,17,17,17,16,16,16,16,16,15,15,15,15,15,14,14,14,14,14,14,13,13,13,13,13,13,12,12,12,12,12,12,11,11,11,11,11,11,10,10,10,10,10,10,9,9,9,9,9,9,9,8,8,8,8,8,8,8,8,7,7,7,7,7,7,7,7,6,6,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,9,9,9,9,9,9,9,10,10,10,10,10,10,10,11,11,11,11,11,11,12,12,12,12,12,12,13,13,13,13,13,13,14,14,14,14,14,15,15,15,15,15,15,16,16,16,16,16,17,17,17,17,17,18,18,18,18,18,19,19,19,19,19,20,20,20,20,21,21,21,21,21,22,22,22,22,23,23,23,23,24,24,24,24,24,25,25,25,25,26,26,26,26,27,27,27,27,28,28,28,28,29,29,29,30,30,30,30,31,31,31,31,32,32,32,32,33,33,33,34,34,34,34,35,35,35,36,36,36,36,37,37,37,38,38,38,39,39,39,39,40,40,40,41,41,41,42,42,42,43,43,43,44,44,44,44,45,45,45,46,46,46,47,47,47,48,48,48,49,49,49,50,50,51,51,51,52,52,52,53,53,53,54,54,54,55,55,55,56,56,57,57,57,58,58,58,59,59,59,60,60,61,61,61,62,62,63,63,63,64,64,64,65,65,66,66,66,67,67,68,68,68,69,69,70,70,70,71,71,72,72,72,73,73,74,74,75,75,75,76,76,77,77,78,78,78,79,79,80,80,81,81,81,82,82,83,83,84,84,84,85,85,86,86,87,87,88,88,89,89,89,90,90,91,91,92,92,93,93,94,94,95,95,96,96,96,97,97,98,98,99,99,100,100,101,101,102,102,103,103,104,104,105,105,106,106,107,107,108,108,109,109,110,110,111,111,112,112,113,113,114,114,115,115,116,116,117,117,118,118,119,120,120,121,121,122,122,123,123,124,124,125,125,126,126,127,128,128,129,129,130,130,131,131,132,133,133,134,134,135,135,136,136,137,138,138,139,139,140,140,141,142,142,143,143,144,144,145,146,146,147,147,148,148,149,150,150,151,151,152,153,153,154,154,155,156,156,157,157,158,159,159,160,160,161,162,162,163,163,164,165,165,166,167,167,168,168,169,170,170,171,172,172,173,173,174,175,175,176,177,177,178,179,179,180,180,181,182,182,183,184,184,185,186,186,187,188,188,189,190,190,191,192,192,193,194,194,195,196,196,197,198,198,199,200,200,201,202,202,203,204,204,205,206,206,207,208,208,209,210,211,211,212,213,213,214,215,215,216,217,218,218,219,220,220,221,222,222,223,224,225,225,226,227,227,228,229,230,230,231,232,233,233,234,235,235,236,237,238,238,239,240,241,241,242,243,244,244,245,246,247,247,248,249,250,250,251,252,253,253,254,255,256,256,257,258,259,259,260,261,262,262,263,264,265,266,266,267,268,269,269,270,271,272,273,273,274,275,276,276,277,278,279,280,280,281,282,283,284,284,285,286,287,288,288,289,290,291,292,292,293,294,295,296,296,297,298,299,300,301,301,302,303,304,305,305,306,307,308,309,310,310,311,312,313,314,315,315,316,317,318,319,320,320,321,322,323,324,325,326,326,327,328,329,330,331,332,332,333,334,335,336,337,338,338,339,340,341,342,343,344,344,345,346,347,348,349,350,351,351,352,353,354,355,356,357,358,359,359,360,361,362,363,364,365,366,367,367,368,369,370,371,372,373,374,375,376,376,377,378,379,380,381,382,383,384,385,386,387,387,388,389,390,391,392,393,394,395,396,397,398,399,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,547,548,549,550,551,552,553,554,555,556,557,558,559,561,562,563,564,565,566,567,568,569,570,571,572,574,575,576,577,578,579,580,581,582,583,584,586,587,588,589,590,591,592,593,594,595,597,598,599,600,601,602,603,604,605,607,608,609,610,611,612,613,614,616,617,618,619,620,621,622,623,625,626,627,628,629,630,631,632,634,635,636,637,638,639,640,642,643,644,645,646,647,648,650,651,652,653,654,655,656,658,659,660,661,662,663,665,666,667,668,669,670,672,673,674,675,676,677,679,680,681,682,683,684,686,687,688,689,690,691,693,694,695,696,697,698,700,701,702,703,704,706,707,708,709,710,712,713,714,715,716,717,719,720,721,722,723,725,726,727,728,729,731,732,733,734,735,737,738,739,740,741,743,744,745,746,748,749,750,751,752,754,755,756,757,758,760,761,762,763,765,766,767,768,770,771,772,773,774,776,777,778,779,781,782,783,784,786,787,788,789,790,792,793,794,795,797,798,799,800,802,803,804,805,807,808,809,810,812,813,814,815,817,818,819,820,822,823,824,825,827,828,829,830,832,833,834,836,837,838,839,841,842,843,844,846,847,848,850,851,852,853,855,856,857,858,860,861,862,864,865,866,867,869,870,871,873,874,875,876,878,879,880,882,883,884,885,887,888,889,891,892,893,895,896,897,898,900,901,902,904,905,906,908,909,910,911,913,914,915,917,918,919,921,922,923,925,926,927,928,930,931,932,934,935,936,938,939,940,942,943,944,946,947,948,950,951,952,954,955,956,958,959,960,962,963,964,966,967,968,970,971,972,974,975,976,978,979,980,982,983,984,986,987,988,990,991,992,994,995,996,998,999,1000,1002,1003,1004,1006,1007,1009,1010,1011,1013,1014,1015,1017,1018,1019,1021,1022,1023,1025,1026,1028,1029,1030,1032,1033,1034,1036,1037,1038,1040,1041,1043,1044,1045,1047,1048,1049,1051,1052,1053,1055,1056,1058,1059,1060,1062,1063,1064,1066,1067,1069,1070,1071,1073,1074,1076,1077,1078,1080,1081,1082,1084,1085,1087,1088,1089,1091,1092,1094,1095,1096,1098,1099,1100,1102,1103,1105,1106,1107,1109,1110,1112,1113,1114,1116,1117,1119,1120,1121,1123,1124,1126,1127,1128,1130,1131,1133,1134,1135,1137,1138,1140,1141,1143,1144,1145,1147,1148,1150,1151,1152,1154,1155,1157,1158,1159,1161,1162,1164,1165,1167,1168,1169,1171,1172,1174,1175,1176,1178,1179,1181,1182,1184,1185,1186,1188,1189,1191,1192,1194,1195,1196,1198,1199,1201,1202,1204,1205,1206,1208,1209,1211,1212,1214,1215,1216,1218,1219,1221,1222,1224,1225,1226,1228,1229,1231,1232,1234,1235,1237,1238,1239,1241,1242,1244,1245,1247,1248,1250,1251,1252,1254,1255,1257,1258,1260,1261,1263,1264,1266,1267,1268,1270,1271,1273,1274,1276,1277,1279,1280,1281,1283,1284,1286,1287,1289,1290,1292,1293,1295,1296,1298,1299,1300,1302,1303,1305,1306,1308,1309,1311,1312,1314,1315,1317,1318,1320,1321,1322,1324,1325,1327,1328,1330,1331,1333,1334,1336,1337,1339,1340,1342,1343,1345,1346,1347,1349,1350,1352,1353,1355,1356,1358,1359,1361,1362,1364,1365,1367,1368,1370,1371,1373,1374,1376,1377,1379,1380,1382,1383,1384,1386,1387,1389,1390,1392,1393,1395,1396,1398,1399,1401,1402,1404,1405,1407,1408,1410,1411,1413,1414,1416,1417,1419,1420,1422,1423,1425,1426,1428,1429,1431,1432,1434,1435,1437,1438,1440,1441,1443,1444,1446,1447,1449,1450,1452,1453,1455,1456,1458,1459,1461,1462,1464,1465,1467,1468,1470,1471,1473,1474,1476,1477,1479,1480,1482,1483,1485,1486,1488,1489,1491,1492,1494,1495,1497,1498,1500,1501,1503,1505,1506,1508,1509,1511,1512,1514,1515,1517,1518,1520,1521,1523,1524,1526,1527,1529,1530,1532,1533,1535,1536,1538,1539,1541,1542,1544,1545,1547,1549,1550,1552,1553,1555,1556,1558,1559,1561,1562,1564,1565,1567,1568,1570,1571,1573,1574,1576,1578,1579,1581,1582,1584,1585,1587,1588,1590,1591,1593,1594,1596,1597,1599,1600,1602,1604,1605,1607,1608,1610,1611,1613,1614,1616,1617,1619,1620,1622,1623,1625,1627,1628,1630,1631,1633,1634,1636,1637,1639,1640,1642,1643,1645,1647,1648,1650,1651,1653,1654,1656,1657,1659,1660,1662,1664,1665,1667,1668,1670,1671,1673,1674,1676,1677,1679,1680,1682,1684,1685,1687,1688,1690,1691,1693,1694,1696,1697,1699,1701,1702,1704,1705,1707,1708,1710,1711,1713,1715,1716,1718,1719,1721,1722,1724,1725,1727,1728,1730,1732,1733,1735,1736,1738,1739,1741,1742,1744,1746,1747,1749,1750,1752,1753,1755,1756,1758,1760,1761,1763,1764,1766,1767,1769,1770,1772,1774,1775,1777,1778,1780,1781,1783,1784,1786,1788,1789,1791,1792,1794,1795,1797,1798,1800,1802,1803,1805,1806,1808,1809,1811,1812,1814,1816,1817,1819,1820,1822,1823,1825,1827,1828,1830,1831,1833,1834,1836,1837,1839,1841,1842,1844,1845,1847,1848,1850,1852,1853,1855,1856,1858,1859,1861,1862,1864,1866,1867,1869,1870,1872,1873,1875,1877,1878,1880,1881,1883,1884,1886,1888,1889,1891,1892,1894,1895,1897,1898,1900,1902,1903,1905,1906,1908,1909,1911,1913,1914,1916,1917,1919,1920,1922,1924,1925,1927,1928,1930,1931,1933,1935,1936,1938,1939,1941,1942,1944,1945,1947,1949,1950,1952,1953,1955,1956,1958,1960,1961,1963,1964,1966,1967,1969,1971,1972,1974,1975,1977,1978,1980,1982,1983,1985,1986,1988,1989,1991,1993,1994,1996,1997,1999,2000,2002,2004,2005,2007,2008,2010,2011,2013,2015,2016,2018,2019,2021,2022,2024,2026,2027,2029,2030,2032,2033,2035,2037,2038,2040,2041,2043,2044,2046,2047
};