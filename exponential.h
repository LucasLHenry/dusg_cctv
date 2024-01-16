const float exptable[256] PROGMEM = {
0.0000,
0.0123,
0.0248,
0.0374,
0.0501,
0.0631,
0.0762,
0.0894,
0.1028,
0.1164,
0.1301,
0.1440,
0.1581,
0.1724,
0.1868,
0.2014,
0.2162,
0.2311,
0.2463,
0.2616,
0.2772,
0.2929,
0.3088,
0.3249,
0.3412,
0.3577,
0.3744,
0.3913,
0.4085,
0.4258,
0.4433,
0.4611,
0.4791,
0.4973,
0.5157,
0.5344,
0.5533,
0.5724,
0.5917,
0.6113,
0.6311,
0.6512,
0.6715,
0.6921,
0.7129,
0.7340,
0.7554,
0.7770,
0.7988,
0.8210,
0.8434,
0.8661,
0.8890,
0.9123,
0.9358,
0.9596,
0.9838,
1.0082,
1.0329,
1.0579,
1.0832,
1.1089,
1.1348,
1.1611,
1.1877,
1.2146,
1.2419,
1.2695,
1.2974,
1.3257,
1.3543,
1.3833,
1.4126,
1.4423,
1.4723,
1.5028,
1.5336,
1.5648,
1.5963,
1.6283,
1.6606,
1.6934,
1.7265,
1.7601,
1.7940,
1.8284,
1.8632,
1.8985,
1.9341,
1.9703,
2.0068,
2.0438,
2.0813,
2.1192,
2.1576,
2.1965,
2.2358,
2.2756,
2.3159,
2.3567,
2.3980,
2.4399,
2.4822,
2.5251,
2.5684,
2.6124,
2.6568,
2.7018,
2.7474,
2.7935,
2.8402,
2.8874,
2.9353,
2.9837,
3.0328,
3.0824,
3.1326,
3.1835,
3.2350,
3.2871,
3.3399,
3.3933,
3.4473,
3.5021,
3.5575,
3.6136,
3.6703,
3.7278,
3.7860,
3.8449,
3.9045,
3.9649,
4.0260,
4.0879,
4.1505,
4.2139,
4.2780,
4.3430,
4.4087,
4.4753,
4.5427,
4.6109,
4.6800,
4.7499,
4.8206,
4.8923,
4.9648,
5.0382,
5.1125,
5.1877,
5.2639,
5.3410,
5.4190,
5.4980,
5.5780,
5.6589,
5.7409,
5.8239,
5.9078,
5.9929,
6.0789,
6.1660,
6.2542,
6.3435,
6.4339,
6.5254,
6.6180,
6.7118,
6.8067,
6.9027,
7.0000,
7.0985,
7.1981,
7.2990,
7.4012,
7.5046,
7.6092,
7.7152,
7.8224,
7.9310,
8.0409,
8.1522,
8.2648,
8.3789,
8.4943,
8.6111,
8.7294,
8.8492,
8.9704,
9.0931,
9.2173,
9.3430,
9.4703,
9.5992,
9.7296,
9.8617,
9.9954,
10.1307,
10.2677,
10.4063,
10.5467,
10.6888,
10.8327,
10.9783,
11.1257,
11.2750,
11.4260,
11.5790,
11.7338,
11.8905,
12.0491,
12.2097,
12.3723,
12.5369,
12.7035,
12.8721,
13.0429,
13.2157,
13.3906,
13.5677,
13.7470,
13.9285,
14.1123,
14.2982,
14.4865,
14.6771,
14.8701,
15.0654,
15.2631,
15.4632,
15.6659,
15.8710,
16.0786,
16.2888,
16.5016,
16.7170,
16.9350,
17.1557,
17.3792,
17.6054,
17.8343,
18.0661,
18.3008,
18.5383,
18.7788,
19.0222,
19.2686,
19.5181,
19.7706,
20.0262,
20.2850,
20.5470,
20.8121,
21.0806,
21.3523,
21.6274,
21.9059,
22.1878,
22.4732,
22.7621,
23.0545,
23.3505,
23.6502,
23.9536,
24.2607,
24.5716,
24.8863,
25.2049,
25.5274,
25.8539,
26.1844,
26.5189,
26.8576,
27.2005,
27.5475,
27.8989,
28.2545,
28.6146,
28.9790,
29.3480,
29.7215,
30.0996,
30.4823,
30.8698,
31.2620,
31.6591,
32.0610,
32.4679,
32.8798,
33.2968,
33.7188,
34.1461,
34.5787,
35.0166,
35.4598,
35.9085,
36.3628,
36.8226,
37.2881,
37.7593,
38.2363,
38.7192,
39.2080,
39.7029,
40.2038,
40.7109,
41.2243,
41.7439,
42.2700,
42.8025,
43.3416,
43.8873,
44.4397,
44.9990,
45.5651,
46.1382,
46.7183,
47.3056,
47.9001,
48.5019,
49.1111,
49.7278,
50.3522,
50.9842,
51.6239,
52.2716,
52.9272,
53.5909,
54.2627,
54.9429,
55.6314,
56.3283,
57.0339,
57.7481,
58.4711,
59.2031,
59.9440,
60.6940,
61.4533,
62.2219,
63.0000,
63.7877,
64.5850,
65.3922,
66.2093,
67.0364,
67.8738,
68.7214,
69.5795,
70.4481,
71.3274,
72.2176,
73.1187,
74.0308,
74.9543,
75.8890,
76.8353,
77.7932,
78.7630,
79.7446,
80.7383,
81.7443,
82.7627,
83.7935,
84.8371,
85.8935,
86.9629,
88.0455,
89.1414,
90.2508,
91.3738,
92.5107,
93.6615,
94.8265,
96.0059,
97.1997,
98.4083,
99.6317,
100.8702,
102.1239,
103.3931,
104.6779,
105.9785,
107.2951,
108.6279,
109.9771,
111.3429,
112.7255,
114.1251,
115.5420,
116.9763,
118.4282,
119.8980,
121.3859,
122.8922,
124.4169,
125.9604,
127.5230,
129.1047,
130.7059,
132.3268,
133.9677,
135.6288,
137.3103,
139.0125,
140.7356,
142.4800,
144.2458,
146.0334,
147.8429,
149.6748,
151.5292,
153.4063,
155.3066,
157.2303,
159.1777,
161.1490,
163.1446,
165.1648,
167.2098,
169.2799,
171.3756,
173.4970,
175.6446,
177.8186,
180.0193,
182.2472,
184.5024,
186.7854,
189.0965,
191.4361,
193.8044,
196.2019,
198.6289,
201.0857,
203.5728,
206.0905,
208.6392,
211.2193,
213.8311,
216.4750,
219.1515,
221.8609,
224.6037,
227.3803,
230.1910,
233.0363,
235.9166,
238.8323,
241.7840,
244.7720,
247.7967,
250.8587,
253.9583,
257.0961,
260.2726,
263.4881,
266.7432,
270.0383,
273.3740,
276.7508,
280.1691,
283.6295,
287.1324,
290.6785,
294.2682,
297.9021,
301.5808,
305.3047,
309.0744,
312.8905,
316.7536,
320.6642,
324.6230,
328.6305,
332.6873,
336.7940,
340.9513,
345.1597,
349.4200,
353.7326,
358.0984,
362.5178,
366.9917,
371.5206,
376.1053,
380.7464,
385.4446,
390.2006,
395.0151,
399.8889,
404.8227,
409.8172,
414.8732,
419.9914,
425.1726,
430.4176,
435.7271,
441.1020,
446.5430,
452.0509,
457.6267,
463.2711,
468.9849,
474.7691,
480.6244,
486.5518,
492.5522,
498.6264,
504.7754,
511.0000
};