minor change for Conti_ARS408.dbc compare to Conti_ARS408_origin.dbc:
2047.5 => 2047   //  related to the filter configuration of the max/min distance  (FilterState_Min_Y, FilterState_Min_X, FilterState_Max_Y, FilterState_Max_X)
511.5 => 511   // related with the Cluster_DistLat, this may cause this signal unaccurate 


(The change is aimed to solve this error below by cantools: "invalid literal for int() with base 10: '2047.5'")
