function scenario = AppleHill()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.10 (R2021a) and Automated Driving Toolbox 3.3 (R2021a).
% Generated on: 12-Aug-2021 08:13:48

% Construct a drivingScenario object.
scenario = drivingScenario('GeographicReference', [42.301265 -71.347315 0], ...
    'VerticalAxis', 'Y');

% Add all road segments
roadCenters = [167.786 41.0124 -0.00233573;
    121.931 34.3689 -0.00125652];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [121.931 34.3689 -0.00125652;
    116.547 41.2224 -0.0011967;
    114.122 43.9438 -0.00117114;
    108.482 54.8517 -0.00115753];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '567741171');

roadCenters = [108.482 54.8517 -0.00115753;
    106.825 42.3997 -0.00103446;
    105.275 39.3672 -0.000989243;
    100.344 31.1806 -0.000864509];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '567741171');

roadCenters = [108.482 54.8517 -0.00115753;
    101.431 124.732 -0.00202757];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '567741165');

roadCenters = [167.786 41.0124 -0.00233573;
    161.701 47.3882 -0.00222306;
    154.931 59.6956 -0.00215881];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '567741162');

roadCenters = [154.931 59.6956 -0.00215881;
    151.17 74.7912 -0.0022282;
    140.887 89.2202 -0.00217905;
    110.138 116.434 -0.00201456;
    101.431 124.732 -0.00202757];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '567741168');

roadCenters = [116.82 -18.1605 -0.0010941;
    118.526 -1.23198 -0.00109975;
    118.469 2.98903 -0.00109926;
    118.229 7.65435 -0.00109873;
    117.767 11.631 -0.00109622;
    116.135 23.1387 -0.00109776];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Walnut Street');

roadCenters = [121.931 34.3689 -0.00125652;
    100.344 31.1806 -0.000864509];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [100.344 31.1806 -0.000864509;
    24.3253 20.1165 -7.81079e-05];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [-125.849 -12.0065 -0.00125102;
    115.5 23.1 -0.00109776];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [-68.1519 -821.496 -0.0533819;
    -68.4402 -777.375 -0.0478429;
    -67.6729 -742.13 -0.0436273;
    -63.713 -584.009 -0.0271128;
    -63.0365 -557.439 -0.0247234;
    -60.6774 -491.414 -0.01926;
    -59.1432 -440.95 -0.0155493;
    -57.5926 -403.75 -0.0130664;
    -56.6771 -396.063 -0.0125752;
    -55.234 -387.01 -0.0120057;
    -53.0239 -380.968 -0.0116223;
    -50.3521 -375.98 -0.0113041;
    -45.7341 -370.937 -0.0109735;
    -39.137 -364.673 -0.0105676;
    -28.1281 -355.731 -0.0100036;
    46.5417 -298.236 -0.00715727;
    60.0984 -287.461 -0.00677464;
    70.2412 -279.497 -0.00652337;
    76.8051 -272.854 -0.00631067;
    82.1403 -266.601 -0.00611201;
    86.2881 -259.269 -0.00586381;
    89.504 -251.627 -0.00560132;
    91.7963 -243.774 -0.0053282;
    93.4042 -236.098 -0.00506214;
    96.0016 -222.591 -0.0046139;
    98.03 -209.85 -0.00421186;
    101.946 -174.016 -0.00319249;
    110.826 -77.5324 -0.00143365;
    116.82 -18.1605 -0.0010941];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Walnut Street');

roadCenters = [24.325 20.116 -7.8108e-05;
    -37.791 10.986 -0.00012127];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [-37.791 10.986 -0.00012127;
    -69.669 6.2541 -0.000383];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [-69.6694 6.2541 -0.000383;
    -145.144 -7.77404 -0.00165373];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [-187.422 -62.8794 -0.00306016;
    -186.647 -54.2597 -0.00295813;
    -185.204 -46.6175 -0.00285556;
    -182.689 -40.9081 -0.00274387;
    -179.324 -36.0207 -0.002619;
    -173.807 -31.3999 -0.00244204;
    -166.683 -27.6789 -0.00223489;
    -156.656 -23.5804 -0.00196461;
    -146.992 -20.3259 -0.00172368;
    -125.849 -12.0065 -0.00125102];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');

roadCenters = [-145.144 -7.77404 -0.00165373;
    -157.06 -9.52883 -0.00193797];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [-157.06 -9.52883 -0.00193797;
    -204.02 -16.4256 -0.00327928];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [-202.7 -23.8 0;
    -202.7 -23.8 0;
    -125.849 -12.0065 -0.00125102];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [115.101 23.0435 -0.00125102;
    357.085 58.1887 -0.00109776];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

roadCenters = [334.7 67.5 -0.00125102;
    167.69 41.05 -0.00109776];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Worcester Street');

