model AZEAT_FortunaCrane_Short_ControllerGroup
  //=============
  /*This Model is used as the validation case*/
  /*No FuelCell is implmented in the model */
  //=============
  // this model is set up for max 6 engine  in total .
  // engine is divived  into  2 groups diesel and methanol
  // note: The model has proper controllers , the correct  BSFC curve calculation,
  // the model can not handle  battery priority yet, it also can not run fuel cell
  // battery can only be used for loading leveling
  // have the option to manually define the optimal upper bound and lower bound
  // External Model Repository for Local AZEAT_FortunaCrane_Short_ControllerGroup Use
  // Modified Controller Logic, if battery SOC smaller than  0.25 (instead of 0.2 which it never dip below of) , then we conside the battery unable to produce any power
  // Modified Controller Logic, if demand is smaller than the idling rate of prioritised generator, the generator check if there is capable battery, if there is, then battery will handle the demand alone. Hence a significant fuel save
  // Fixed bug where Manually input higher and lower bound is reversied
  // 13th OCT Modifiy the controller logic, in Node 2, and Node 2.5, changed Node condition and generator output when battery needs charging.
  // 17th OCT Modeify the controller logic, in every node invovles P_diesel_idle now uses DieselGeneratorCount*P_diesel_idle
  // 17th OCT Resreucture the Node 2, Remove Node 2.5, now P_net = 0 to P_net  = LowerBound is a whole range

  /*Master Controller*/

  model OptimalFuelConsumptionBoundsCalculator_Polynomial
    import Modelica.Blocks.Interfaces.RealOutput;
    // Polynomial coefficients (a is constant term)
    parameter Real a = 0;
    parameter Real b = 0;
    parameter Real c = 0;
    parameter Real d = 0;
    parameter Real e = 0;
    parameter Real f = 0;
    parameter Real g = 0;
    // Range of interest
    parameter Real P_idle = 0;
    parameter Real P_rat = 1;
    // Number of points for search
    parameter Integer N = 1000;
    // Output connectors
    RealOutput x_min annotation(
      Placement(transformation(origin = {-110, 50}, extent = {{10, -10}, {-10, 10}}), iconTransformation(origin = {-110, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    RealOutput x_110_low annotation(
      Placement(transformation(origin = {-110, 10}, extent = {{10, -10}, {-10, 10}}), iconTransformation(origin = {-110, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    RealOutput x_110_high annotation(
      Placement(transformation(origin = {-110, -30}, extent = {{10, -10}, {-10, 10}}), iconTransformation(origin = {-110, -70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  protected
    Real x[N + 1];
    Real y[N + 1];
    Real y_min;
    Real y_110;
    Integer i_min;
    Integer i_110_low;
    Integer i_110_high;
  algorithm
// Discretize the range
    for i in 1:N + 1 loop
      x[i] := P_idle + (P_rat - P_idle)*(i - 1)/N;
      y[i] := a + b*x[i] + c*x[i]^2 + d*x[i]^3 + e*x[i]^4 + f*x[i]^5 + g*x[i]^6;
    end for;
// Find minimum
    y_min := y[1];
    i_min := 1;
    for i in 2:N + 1 loop
      if y[i] < y_min then
        y_min := y[i];
        i_min := i;
      end if;
    end for;
    x_min := x[i_min];
    y_110 := 1.1*y_min;
// Find x values where y crosses 110% of min (first and last crossing)
    for i in 2:N + 1 loop
      if (y[i - 1] < y_110 and y[i] >= y_110) or (y[i - 1] > y_110 and y[i] <= y_110) then
        if i_110_low == 0 then
          i_110_low := i;
        else
          i_110_high := i;
        end if;
      end if;
    end for;
    x_110_low := x[i_110_low];
    x_110_high := x[i_110_high];
    annotation(
      Icon(graphics = {Text(origin = {-68, 6}, extent = {{-24, 62}, {24, -62}}, textString = "Y"), Text(origin = {-25, 4}, extent = {{-65, 32}, {65, -32}}, textString = "="), Text(origin = {48, 2}, extent = {{-48, 52}, {48, -52}}, textString = "bX+a")}));
  end OptimalFuelConsumptionBoundsCalculator_Polynomial;

  model OptimalFuelConsumptionBoundsCalculator_ExternalTable
    parameter Real minimal_bsfc_percentage = 0.04;
    parameter Boolean manually_define_bounds = true;
    parameter Real manual_defined_upperBound = 400;
    parameter Real manual_defined_lowerBound = 200;
    parameter Real BSFC_Curve_Default[:, 2] = [80, 263.61; 160, 250.95; 240, 243.83; 320, 238.90; 400, 235.15; 480, 232.12; 560, 229.60; 640, 227.43; 720, 225.54; 800, 223.86];
    parameter Real min_bsfc = min(BSFC_Curve_Default[:, 2]);
    parameter Integer minIndex = Modelica.Math.Vectors.find(min_bsfc, BSFC_Curve_Default[:, 2]);
    Modelica.Blocks.Interfaces.RealOutput bsfc_min_out annotation(
      Placement(transformation(origin = {-110, 50}, extent = {{10, -10}, {-10, 10}}), iconTransformation(origin = {-110, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    parameter Integer n = size(BSFC_Curve_Default, 1);
    parameter Integer nLeft = minIndex;
    parameter Integer nRight = n - minIndex + 1;
    parameter Real upper_optim_bsfc = min_bsfc*(1 + minimal_bsfc_percentage);
    // Syntax, 2 d array slicing 2D_ArrayName[startIndex : steps :endIndex, {firstColumnToBe Sliced, secondColumnToBeSliced}]
    //                                            BSFC_Curve_Default [     minIndex:-1:1,     {2                                    ,                                         1}]
    parameter Real BSFC_Curve_Reverse_Left[nLeft, 2] = BSFC_Curve_Default[minIndex:-1:1, {2, 1}];
    parameter Real BSFC_Curve_Reverse_Right[nRight, 2] = BSFC_Curve_Default[minIndex:n, {2, 1}];
    parameter Real BSFC_Curve_Reverse[n, 2] = [BSFC_Curve_Default[:, 2], BSFC_Curve_Default[:, 1]];
    Modelica.Blocks.Tables.CombiTable1Ds table_to_calc_pwr_lower(table = BSFC_Curve_Reverse_Left, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) annotation(
      Placement(transformation(origin = {10, 40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Tables.CombiTable1Ds table_to_calc_pwr_higher(table = BSFC_Curve_Reverse_Right, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) annotation(
      Placement(transformation(origin = {10, -8}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Sources.RealExpression max_optim_bsfc(y = upper_optim_bsfc) annotation(
      Placement(transformation(origin = {90, 20}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Interfaces.RealOutput pwr_lower_band annotation(
      Placement(transformation(origin = {-110, 10}, extent = {{10, -10}, {-10, 10}}), iconTransformation(origin = {-110, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Interfaces.RealOutput pwr_higher_band annotation(
      Placement(transformation(origin = {-110, -30}, extent = {{10, -10}, {-10, 10}}), iconTransformation(origin = {-110, -70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Sources.RealExpression realExpressionUpperBound(y = manual_defined_upperBound) annotation(
      Placement(transformation(origin = {-30, 76}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.RealExpression realExpressionLowerBound(y = manual_defined_lowerBound) annotation(
      Placement(transformation(origin = {-28, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  equation
    bsfc_min_out = min_bsfc;
    connect(max_optim_bsfc.y, table_to_calc_pwr_lower.u) annotation(
      Line(points = {{80, 20}, {40, 20}, {40, 40}, {22, 40}}, color = {0, 0, 127}));
    connect(max_optim_bsfc.y, table_to_calc_pwr_higher.u) annotation(
      Line(points = {{80, 20}, {42, 20}, {42, -8}, {22, -8}}, color = {0, 0, 127}));
    if manually_define_bounds then
      connect(realExpressionLowerBound.y, pwr_lower_band) annotation(
        Line(points = {{-40, 56}, {-46, 56}, {-46, -30}, {-110, -30}}, color = {0, 0, 127}));
      connect(realExpressionUpperBound.y, pwr_higher_band) annotation(
        Line(points = {{-40, 76}, {-62, 76}, {-62, 10}, {-110, 10}}, color = {0, 0, 127}));
    else
      connect(table_to_calc_pwr_lower.y[1], pwr_lower_band) annotation(
        Line(points = {{0, 40}, {-56, 40}, {-56, 10}, {-110, 10}}, color = {0, 0, 127}));
      connect(table_to_calc_pwr_higher.y[1], pwr_higher_band) annotation(
        Line(points = {{0, -8}, {-48, -8}, {-48, -30}, {-110, -30}}, color = {0, 0, 127}));
    end if;
    annotation(
      Icon(graphics = {Rectangle(fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}), Rectangle(origin = {-39, 53}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {-39, 23}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {-39, -7}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {33, -37}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {-39, -63}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {33, 53}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {33, 23}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {33, -7}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {-39, -37}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}}), Rectangle(origin = {33, -63}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 9}, {21, -9}})}));
  end OptimalFuelConsumptionBoundsCalculator_ExternalTable;

  model MasterControllerSingleBattery
    //External Models For Local MasterController Use
    /*BatteryDispatchableControlBase*/

    partial model BatteryDispatchableControlBase "Base interface for battery/dispatchable energy sources controller."
      parameter Integer n = 1 "Number of dispatchable diesel units";
      parameter Integer alt_n = 1 "Number of dispatchable alt_fuel units ";
      parameter Integer acutal_diesel_n = 1 "Number of actual dispatchable  diesel units ";
      parameter Integer actual_altFuel_n = 1 "Number of  actual dispatchable alt fuel units";
      parameter Real dieselP_start = 0 "Start power of the genset" annotation(
        Dialog(tab = "Initialization"));
      parameter Real batteryP_start = 0 "Start power of the battery" annotation(
        Dialog(tab = "Initialization"));
      parameter Real SOC_max = 0.9 "Maximum state of charge" annotation(
        Dialog(group = "Operational constraints"));
      parameter Real SOC_min = 0.1 "Minimum state of charge" annotation(
        Dialog(group = "Operational constraints"));
      parameter Modelica.Units.SI.Power P_charging_max = 2e5 "Maximum charge rate of the battery" annotation(
        Dialog(group = "Operational constraints"));
      parameter Modelica.Units.SI.Power P_charging_min = -P_charging_max "Maximum discharge rate of the battery" annotation(
        Dialog(group = "Operational constraints"));
      parameter Real P_max[n] = {10} "Maximal power of the gensets, in decreasing priority" annotation(
        Dialog(group = "Operational constraints"));
      parameter Real P_max_alt[alt_n] = {10} "Maximal  power of the alt_fuel gensets, in decreasing order" annotation(
        Dialog(group = "Operational constraints"));
      Modelica.Blocks.Interfaces.RealOutput P_alt_dispatch[alt_n] annotation(
        Placement(transformation(origin = {110, -72}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, -74}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput P_dispatch[n] annotation(
        Placement(transformation(extent = {{100.23233486262458, -51.58794040978954}, {120.23233486262458, -31.58794040978954}}, rotation = 0.0, origin = {0.0, 0.0})));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid), Text(extent = {{-38, 26}, {38, -30}}, lineColor = {255, 255, 255}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, textString = "C"), Text(extent = {{-100, -110}, {100, -130}}, lineColor = {0, 0, 0}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, textString = "%name")}));
    end BatteryDispatchableControlBase;

    /*New Multi Controller  Core*/

    model MethanolBatteryDieselDebug
      //External Function Repository for Local Use//
      /*cubicStep function*/
    
      function cubicStep "Cubic step function"
        input Real tau "Abcissa";
        output Real y "Value";
      algorithm
        y := if tau < 0 then 0 else (if tau > 1 then 1 else (3 - 2*tau)*tau^2);
      end cubicStep;
    
      /*minLocal function*/
    
      function minLocal
        input Real a;
        input Real b;
        output Real result;
      algorithm
        result := if a < b then a else b;
      end minLocal;
    
      /*maxLocal function*/
    
      function maxLocal
        input Real a;
        input Real b;
        output Real result;
      algorithm
        result := if a > b then a else b;
      end maxLocal;
    
      parameter Real SOC_max = 0.9 "Max State of Charge" annotation(
        Dialog(group = "Battery"));
      parameter Real SOC_min = 0.2 "Min State of Charge" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_charging_max = 2e5 "Max Charing Rate" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_charging_min = -P_charging_max "Maximum discharge rate" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_rated_diesel_gen "Generator max output" annotation(
        Dialog(group = "Diesel generator"));
      parameter Modelica.Units.SI.Power P_idling_diesel_gen "Diesel Generator  Idle Output " annotation(
        Dialog(group = "Diesel generator"));
      parameter Modelica.Units.SI.Power P_rated_alt_gen "Alternative Fuel Generator max_output " annotation(
        Dialog(group = "All generators"));
      parameter Modelica.Units.SI.Power P_idling_alt_gen "Alternative Fuel Generator Output " annotation(
        Dialog(group = "All generators"));
      parameter Modelica.Units.SI.Power P_ref_FC_max_rate;
      parameter Modelica.Units.SI.Power P_rated_combined_gen "Combined avalible generator output" annotation(
        Dialog(group = "All generators"));
      parameter Modelica.Units.SI.Power P_rated_battery = 15000 "Max Output of  Battery Stack ";
      parameter Modelica.Units.SI.Power DieselGenChargingRate = 1000 "Diesel charging rate to battery, measured in Watt" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      parameter Modelica.Units.SI.Power AlthFuelGenChargingRate = 1000 "Alt Fuel Generator chariging rate to battery, measured  in Watt" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      parameter Integer DieselGeneratorCount = 1;
      parameter Integer AltFuelGeneratorCount = 1;
      parameter Real dieselP_start = 0 "Initital power by generator";
      parameter Real altFuelP_start = 0 "Initial power by alternative fuel generator";
      parameter Real batteryP_start = 0 "Initial power by battery";
      parameter Real fuelcellP_start = 0 "Initial Power of fuel cell";
      Modelica.Units.SI.Power P_net = P_load_1 "Power Deffciecy is (- load)";
      //output Modelica.Units.SI.Power P_surplus_1 "Surplus Power for Battery 1";
      Modelica.Blocks.Interfaces.RealInput P_renew_1 "Power produced by renewable energy source 1 (photovoltaics, wind, hydro) in W" annotation(
        Placement(transformation(origin = {-50, 50}, extent = {{-70, 10}, {-50, 30}}), iconTransformation(origin = {-50, 30}, extent = {{-70, 10}, {-50, 30}})));
      Modelica.Blocks.Interfaces.RealInput P_load_1 "Load power in W" annotation(
        Placement(transformation(origin = {-220, 10}, extent = {{100, 30}, {120, 50}}), iconTransformation(origin = {-220, 48}, extent = {{100, 30}, {120, 50}})));
      Modelica.Blocks.Interfaces.RealInput SOC_1(min = SOC_min, max = SOC_max) "State of charge in p.u." annotation(
        Placement(transformation(origin = {-110, 32}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput P_battery_1(start = batteryP_start) "Battery_1 charging power" annotation(
        Placement(transformation(origin = {0, 10}, extent = {{100, 30}, {120, 50}}), iconTransformation(origin = {0, 10}, extent = {{100, 30}, {120, 50}})));
      Modelica.Blocks.Interfaces.RealOutput P_dieselGen(start = dieselP_start) "Power produced by diesel generators" annotation(
        Placement(transformation(extent = {{100, -50}, {120, -30}}), iconTransformation(origin = {-40, 22}, extent = {{140, -70}, {168, -42}})));
      Modelica.Blocks.Interfaces.RealOutput P_altFuelGen(start = altFuelP_start) "Power produced by alternative fuel generators" annotation(
        Placement(transformation(origin = {110, -72}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput P_fuelCellGen(start = fuelcellP_start) annotation(
        Placement(transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}})));
      parameter Real smooth_charge_percentage = 0.1;
      Real nodeTracker ;
      Real IceGenChargingRate;
      Real smooth_soc_max;
      //Smooth the charge close to SOC_max,starting at smooth_charge_percentage from SOC_max
      Real smooth_soc_min;
      //Smooth the charge close to SOC_min,starting at smooth_charge_percentage from SOC_min
      Modelica.Blocks.Interfaces.RealInput SOC_tank(min = 0, max = 1) "State of tank" annotation(
        Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270), iconTransformation(origin = {-28, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
      parameter Modelica.Units.SI.Power MinValy annotation(
        Dialog(tab = "Battery Controls"));
      parameter Modelica.Units.SI.Power MaxValu annotation(
        Dialog(tab = "Battery Controls"));
      parameter Real priorityAssignement = 0 "0 represent the Generator Priority, 1 represent then Fuel Cell priority, 2 Represent a Constant Hybrid " annotation(
        Dialog(tab = "Priority Assignment"));
      parameter Real icePriorityAssignment = 0 "0 represent the Diesel Engine Priority, 1 represent the  Alternative Fuel Priority" annotation(
        Dialog(tab = "Priority Assignment"));
      parameter Real iceChargingProrityAssignement = 0 "0 represent the Diesel Generator for charging,  1 Represent the  Alternative  Fuel generator  for charging, 2 for Fuel cell for charging" annotation(
        Dialog(tab = "Priority Assignment"));
      parameter Real dieselEngineShare = 0.5 "Fraction of P_load assigned to ICE Engine" annotation(
        Dialog(tab = "Constant Hybrid Configuration"));
      parameter Real fuelCellShare = 0.5 "Fraction of P_load assigned to Fuel Cell Engine" annotation(
        Dialog(tab = "Constant Hybrid Configuration"));
      parameter Real fuelCellPowerBandCount = 4;
      //Real powerBandSize;
      Boolean chargingStatus (start=false, fixed=true)  "Battery Charging Required ?";
      parameter Real IceChargingSOCmax = 0.7 "Max Battery SOC  to charge battery with ICE engines" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      parameter Real IceChargingSOCmin = 0.1 "Min Battery SOC to charge battery with ICE engines" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      parameter Real IceTankChargingLimit = 0.3 "ICE Fuel Tank minimal SOC limit to charge the battery" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      // Modelica.Blocks.Interfaces.RealOutput P_iceCharging  annotation(
      //Placement(transformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealInput dieselOptimUpperBound annotation(
        Placement(transformation(origin = {-110, -10}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput dieselOptimLowerBound annotation(
        Placement(transformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-10, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput dieselPwrAtMinBSFC annotation(
        Placement(transformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-92, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput altFuelOptimUpperBound annotation(
        Placement(transformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealInput altFuelOptimLowerBound annotation(
        Placement(transformation(origin = {-30, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealInput altFuelPwrAtMinBSFC annotation(
        Placement(transformation(origin = {-70, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}})));
            //PowerChecksLogic parameters
    
          Real Batterypowercheck;
          Real Batterychargecheck;
          Real Generatorcheck;
          Real Zonecheck;
          Real Ifstatementcheck;
          Boolean batterylogiccheck;
          Boolean batterychargelogiccheck;
          Boolean generatorlogiccheck;
          Boolean Zonelogiccheck;
    
    algorithm
      when initial() or SOC_1 <= 0.25 or SOC_1 >= 0.35then
        chargingStatus := 
          if SOC_1 <= 0.25 then true
          elseif SOC_1 >= 0.35 then false
          else pre(chargingStatus); 
      end when;
    equation
      smooth_soc_max = (1 - cubicStep((1 + (SOC_1 - SOC_max)*(1/smooth_charge_percentage))));
      smooth_soc_min = cubicStep((SOC_1 - SOC_min)*(1/smooth_charge_percentage));
       //PowerchecksLogic equations
        Zonecheck = if P_net < DieselGeneratorCount*dieselOptimLowerBound then 1 elseif P_net < DieselGeneratorCount*dieselOptimUpperBound then 2 else 3;
        if Zonecheck == 1 then
          if chargingStatus then
            Batterypowercheck = 0;
            Generatorcheck = DieselGeneratorCount*dieselOptimLowerBound;
            Batterychargecheck = Generatorcheck - P_net;
            Ifstatementcheck = 2;
          else
            Batterypowercheck = if P_rated_battery >= P_net then P_net elseif P_net <= DieselGeneratorCount*P_idling_diesel_gen then 0 else min(P_rated_battery,P_net-DieselGeneratorCount*P_idling_diesel_gen);
            Generatorcheck = if P_rated_battery >= P_net then 0 else max(DieselGeneratorCount*P_idling_diesel_gen,P_net-Batterypowercheck);
            Batterychargecheck = 0;
            Ifstatementcheck = 1;
          end if;
        elseif Zonecheck == 2 then
           Batterypowercheck = 0;
          if chargingStatus then
            Generatorcheck = DieselGeneratorCount*dieselOptimUpperBound;
            Batterychargecheck = Generatorcheck - P_net;
            Ifstatementcheck = 2;
          else
            Generatorcheck = P_net;
            Batterychargecheck = 0;
            Ifstatementcheck = 1;
          end if;
        else 
          if chargingStatus then
            Batterypowercheck = 0;
            Generatorcheck = DieselGeneratorCount*P_rated_diesel_gen;
            Batterychargecheck = Generatorcheck - P_net;
            Ifstatementcheck = 2;
          else
            Batterypowercheck = min(P_rated_battery,P_net-DieselGeneratorCount*dieselOptimUpperBound);
            Generatorcheck = P_net-Batterypowercheck;
            Batterychargecheck = 0;
            Ifstatementcheck = 1;
          end if;
        end if;
        
        batterylogiccheck = if Batterypowercheck == P_battery_1 then true else false;
        batterychargelogiccheck = if Batterychargecheck  == IceGenChargingRate then true else false;
        generatorlogiccheck = if Generatorcheck == P_dieselGen then true else false;
        Zonelogiccheck = if batterylogiccheck == true and generatorlogiccheck == true then true else false;
      if priorityAssignement == 0 then
        if icePriorityAssignment == 0 then
          if iceChargingProrityAssignement == 0 then              
             /*====0,0,0====*/
            if (DieselGeneratorCount*dieselOptimLowerBound < P_net and P_net < DieselGeneratorCount*dieselOptimUpperBound) then
              /*Leaf Node 1 */
              // Prioritise  Diesel, Diesel Charging , Demand within the BSFC Sweet Spot Zone
              P_dieselGen = P_net;
              // Diesel  handels all demand as  long as the demand is within thesweet spot zone
              P_altFuelGen = 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 1;
            elseif (P_net <= DieselGeneratorCount*dieselOptimLowerBound and P_net > 0) then
              /*Leaf Node 2 */
              if (P_rated_battery<1) then 
                  P_dieselGen = P_net;
                  P_altFuelGen = 0;
                  P_battery_1 = 0;
                  P_fuelCellGen = 0; 
                  IceGenChargingRate = 0;
                  nodeTracker = 2;
              else 
                  if (chargingStatus) then 
                      P_dieselGen = DieselGeneratorCount*dieselOptimLowerBound ;
                      P_altFuelGen = 0;
                      P_battery_1 = 0;
                      P_fuelCellGen = 0; 
                      IceGenChargingRate =  DieselGeneratorCount*dieselOptimLowerBound-P_net;
                      nodeTracker = 2.1;
                  else 
                      if (SOC_1> 0.2) then
                        if(P_rated_battery  >= P_net) then 
                            P_dieselGen = 0 ;
                            P_altFuelGen = 0;
                            P_battery_1 = P_net;
                            P_fuelCellGen = 0; 
                            IceGenChargingRate = 0;
                            nodeTracker = 2.2;
                        else
                            if(P_net- P_rated_battery >= DieselGeneratorCount*P_idling_diesel_gen) then
                                P_dieselGen = P_net- P_rated_battery ;
                                P_altFuelGen = 0;
                                P_battery_1 = P_rated_battery;
                                P_fuelCellGen = 0; 
                                IceGenChargingRate = 0;
                                nodeTracker = 2.3;
                            else 
                                P_dieselGen = DieselGeneratorCount*dieselOptimLowerBound ;
                                P_altFuelGen = 0;
                                P_battery_1 = 0;
                                P_fuelCellGen = 0; 
                                IceGenChargingRate = DieselGeneratorCount*dieselOptimLowerBound - P_net;
                                nodeTracker = 2.4;
                            end if;
                        end if;
                      else 
                            P_dieselGen = DieselGeneratorCount*dieselOptimLowerBound ;
                            P_altFuelGen = 0;
                            P_battery_1 = 0;
                            P_fuelCellGen = 0; 
                            IceGenChargingRate = 0;
                            nodeTracker = 2.8;
                      end if;
                  end if;
              end if;       
            //elseif the demand is smaller than the idling power
            elseif (P_net == 0)then 
                P_dieselGen = 0;
                P_altFuelGen = 0;
                P_battery_1 = 0; 
                P_fuelCellGen = 0;
                IceGenChargingRate = 0;  // if charging is requrid andthere is a battery then, charge it wil excess power
                nodeTracker = 2.25;
            elseif (P_net >= DieselGeneratorCount*dieselOptimUpperBound) then
              if (SOC_1 > 0.25 and P_rated_battery > 1) then
                if (P_net > P_rated_diesel_gen) then
                  if (P_net - DieselGeneratorCount*dieselOptimUpperBound <= P_rated_battery) then
                    /*Leaf Node 3 */
                    // Prioritise  Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can Cover the Gap
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                    // diesle generator runs  at upper Bound
                    /*Load Leveling*/
                    P_altFuelGen = 0;
                    // alt gen not need to take part
                    P_battery_1 = (P_net - DieselGeneratorCount*dieselOptimUpperBound)*smooth_soc_min;
                    // unmet power handles byt battery
                    IceGenChargingRate = 0;
                    // no charging action
                    P_fuelCellGen = 0;
                    nodeTracker = 3;
                  else
                    //(P_net - DieselGeneratorCount*dieselOptimUpperBound > P_rated_battery)
                    if (P_rated_alt_gen > 0) then
                      /*Leaf Node 4 */
                      // Prioritise Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Exisit
                      P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                      //diesel generator  runs at upper  Bound
                      P_battery_1 = P_rated_battery*smooth_soc_min;
                      // battery runs at thier max
                      P_altFuelGen = if P_net - DieselGeneratorCount*dieselOptimUpperBound - P_battery_1 > 0 then P_net - DieselGeneratorCount*dieselOptimUpperBound - P_battery_1 else 0;
                      // if there is still unmet  power then handles but alt gen, if not alt gen don't run
                      IceGenChargingRate = 0;
                      // no charing action
                      P_fuelCellGen = 0;
                      nodeTracker = 4;
                    else
                      /*Leaf Node 5 */
                      // Prioritise Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Non - Exisit
                      /*RARE CASE*/
                      P_dieselGen = P_rated_diesel_gen;
                      //diesel gen runs as much as possible
                      P_battery_1 = P_rated_battery*smooth_soc_min;
                      //battery gen runs as much as possible
                      P_altFuelGen = 0;
                      //there is no alt gen
                      IceGenChargingRate = 0;
                      // no charging action
                      P_fuelCellGen = 0;
                      nodeTracker = 5;
                    end if;
                  end if;
                elseif (P_net < P_rated_diesel_gen and P_net >= DieselGeneratorCount*dieselOptimUpperBound) then
                  if (P_rated_battery >= P_net - DieselGeneratorCount*dieselOptimUpperBound) then
                    /*Leaf Node 6*/
                    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound  and  max Rated, Battery Can cover the gap
                    /*Load Leveling */
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                    // Diesel Generator make Upper Bound
                    P_battery_1 = (P_net - DieselGeneratorCount*dieselOptimUpperBound)*smooth_soc_min;
                    // battery  fill the unmet power
                    P_altFuelGen = 0;
                    // no involvoment as the battery cover the demand.
                    IceGenChargingRate = 0;
                    //no charging action
                    P_fuelCellGen = 0;
                    nodeTracker = 6;
                  else
                    /*Leaf Node 7*/
                    // /*Need further detail on allocating the unmet power  */
                    // P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                    P_dieselGen = P_net - P_rated_battery*smooth_soc_min;
                    // diesel generator runs at upper bound
                    P_battery_1 = P_rated_battery*smooth_soc_min;
                    // battery works at hard as possible
                    P_altFuelGen = if (P_rated_alt_gen > 0) then if P_net - P_rated_battery - P_dieselGen > 0 then P_net - P_rated_battery - P_dieselGen else 0 else 0; // if there is alt gen then alt gen handles the unmet power
                    IceGenChargingRate = 0;
                   
                    P_fuelCellGen = 0; // no charging action
                    nodeTracker = 7;
                  end if;
                else
                  P_dieselGen = 130001;/*Default Node 1*/
                  P_battery_1 = 130001;// When demand is  neither  larger than rated power not within higher bound and  rated power
                  P_altFuelGen = 130001;
                  IceGenChargingRate = 0;
                  P_fuelCellGen = 0;
                  nodeTracker = 0.1;
                end if;
              elseif (SOC_1 < 0.25 or P_rated_battery < 1) then
                /*Leaf Node 8*/
                //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery NON - Exist.
                P_dieselGen = if (P_rated_alt_gen > 0) then DieselGeneratorCount*dieselOptimUpperBound else if  P_net < P_rated_diesel_gen then P_net else P_rated_diesel_gen;
                // if there is alt gen then runn at upper bound else run as hard as possible
                P_battery_1 = 0;
                P_altFuelGen = if (P_rated_alt_gen > 0) then P_net - P_dieselGen else 0;
                // if there is  altgen then fill the unmet  else it is 0
                IceGenChargingRate = 0;
                P_fuelCellGen = 0;
                nodeTracker = 8;
              else
                /*Default Node 2*/
                // When the battery is neither exist or non exist, having charge or not having charge
                P_dieselGen = 140001;
                P_altFuelGen = 140001;
                P_battery_1 = 101*smooth_soc_min;
                P_fuelCellGen = 0;
                IceGenChargingRate = 0;
                nodeTracker = 0.2;
              end if;
            else
              /*Default  Node 3 */
              // When the demand is neither with the sweet spot or, higher than the sweet  spot, or lower  than the sweet spot
              P_dieselGen = 100001;
              P_altFuelGen = 100001;
              P_battery_1 = 101*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 0.3;
            end if;
          elseif iceChargingProrityAssignement == 1 then /*====0,0,1====*/
            P_dieselGen = 400001;
            P_altFuelGen = 400001;
            P_battery_1 = 401*smooth_soc_min;
            P_fuelCellGen = 0;
            IceGenChargingRate = 0;
            nodeTracker = 2.9;
          end if;
        elseif icePriorityAssignment == 1 then /*====0,0,1====*/
              P_dieselGen = 400001;
              P_altFuelGen = 400001;
              P_battery_1 = 401*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
        end if;
      elseif priorityAssignement == 1 then /*====1,0,0====*/
        
        P_dieselGen = 400001;
        P_altFuelGen = 400001;
        P_battery_1 = 401*smooth_soc_min;
        P_fuelCellGen = 0;
        IceGenChargingRate = 0;
        nodeTracker = 2.9;
    // ice priority assignement end
      end if;
    // priorityAssignement end
      annotation(
        Icon(graphics = {Ellipse(lineColor = {255, 0, 0}, fillColor = {85, 255, 127}, pattern = LinePattern.DashDot, fillPattern = FillPattern.Horizontal, lineThickness = 1.75, extent = {{-80, 80}, {80, -80}}), Text(origin = {2, 8}, extent = {{-48, 30}, {48, -30}}, textString = "Debug")}));
    end MethanolBatteryDieselDebug;

    model MethanolBatteryDieselControllerCore
      //External Function Repository for Local Use//
      /*cubicStep function*/
    
      function cubicStep "Cubic step function"
        input Real tau "Abcissa";
        output Real y "Value";
      algorithm
        y := if tau < 0 then 0 else (if tau > 1 then 1 else (3 - 2*tau)*tau^2);
      end cubicStep;
    
      /*minLocal function*/
    
      function minLocal
        input Real a;
        input Real b;
        output Real result;
      algorithm
        result := if a < b then a else b;
      end minLocal;
    
      /*maxLocal function*/
    
      function maxLocal
        input Real a;
        input Real b;
        output Real result;
      algorithm
        result := if a > b then a else b;
      end maxLocal;
    
      parameter Real SOC_max = 0.9 "Max State of Charge" annotation(
        Dialog(group = "Battery"));
      parameter Real SOC_min = 0.2 "Min State of Charge" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_charging_max = 2e5 "Max Charing Rate" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_charging_min = -P_charging_max "Maximum discharge rate" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_rated_diesel_gen "Generator max output" annotation(
        Dialog(group = "Diesel generator"));
      parameter Modelica.Units.SI.Power P_idling_diesel_gen "Diesel Generator  Idle Output " annotation(
        Dialog(group = "Diesel generator"));
      parameter Modelica.Units.SI.Power P_rated_alt_gen "Alternative Fuel Generator max_output " annotation(
        Dialog(group = "All generators"));
      parameter Modelica.Units.SI.Power P_idling_alt_gen "Alternative Fuel Generator Output " annotation(
        Dialog(group = "All generators"));
      parameter Modelica.Units.SI.Power P_ref_FC_max_rate;
      parameter Modelica.Units.SI.Power P_rated_combined_gen "Combined avalible generator output" annotation(
        Dialog(group = "All generators"));
      parameter Modelica.Units.SI.Power P_rated_battery = 15000 "Max Output of  Battery Stack ";
      parameter Modelica.Units.SI.Power DieselGenChargingRate = 1000 "Diesel charging rate to battery, measured in Watt" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      parameter Modelica.Units.SI.Power AlthFuelGenChargingRate = 1000 "Alt Fuel Generator chariging rate to battery, measured  in Watt" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      parameter Integer DieselGeneratorCount = 1;
      parameter Integer AltFuelGeneratorCount = 1;
      parameter Real dieselP_start = 0 "Initital power by generator";
      parameter Real altFuelP_start = 0 "Initial power by alternative fuel generator";
      parameter Real batteryP_start = 0 "Initial power by battery";
      parameter Real fuelcellP_start = 0 "Initial Power of fuel cell";
      Modelica.Units.SI.Power P_net = P_load_1 "Power Deffciecy is (- load)";
      //output Modelica.Units.SI.Power P_surplus_1 "Surplus Power for Battery 1";
      Modelica.Blocks.Interfaces.RealInput P_renew_1 "Power produced by renewable energy source 1 (photovoltaics, wind, hydro) in W" annotation(
        Placement(transformation(origin = {-50, 50}, extent = {{-70, 10}, {-50, 30}}), iconTransformation(origin = {-50, 30}, extent = {{-70, 10}, {-50, 30}})));
      Modelica.Blocks.Interfaces.RealInput P_load_1 "Load power in W" annotation(
        Placement(transformation(origin = {-220, 10}, extent = {{100, 30}, {120, 50}}), iconTransformation(origin = {-220, 48}, extent = {{100, 30}, {120, 50}})));
      Modelica.Blocks.Interfaces.RealInput SOC_1(min = SOC_min, max = SOC_max) "State of charge in p.u." annotation(
        Placement(transformation(origin = {-110, 32}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput P_battery_1(start = batteryP_start) "Battery_1 charging power" annotation(
        Placement(transformation(origin = {0, 10}, extent = {{100, 30}, {120, 50}}), iconTransformation(origin = {0, 10}, extent = {{100, 30}, {120, 50}})));
      Modelica.Blocks.Interfaces.RealOutput P_dieselGen(start = dieselP_start) "Power produced by diesel generators" annotation(
        Placement(transformation(extent = {{100, -50}, {120, -30}}), iconTransformation(origin = {-40, 22}, extent = {{140, -70}, {168, -42}})));
      Modelica.Blocks.Interfaces.RealOutput P_altFuelGen(start = altFuelP_start) "Power produced by alternative fuel generators" annotation(
        Placement(transformation(origin = {110, -72}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput P_fuelCellGen(start = fuelcellP_start) annotation(
        Placement(transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}})));
      parameter Real smooth_charge_percentage = 0.1;
      Real nodeTracker ;
      Real IceGenChargingRate;
      Real smooth_soc_max;
      //Smooth the charge close to SOC_max,starting at smooth_charge_percentage from SOC_max
      Real smooth_soc_min;
      //Smooth the charge close to SOC_min,starting at smooth_charge_percentage from SOC_min
      Modelica.Blocks.Interfaces.RealInput SOC_tank(min = 0, max = 1) "State of tank" annotation(
        Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270), iconTransformation(origin = {-28, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
      parameter Modelica.Units.SI.Power MinValy annotation(
        Dialog(tab = "Battery Controls"));
      parameter Modelica.Units.SI.Power MaxValu annotation(
        Dialog(tab = "Battery Controls"));
      parameter Real priorityAssignement = 0 "0 represent the Generator Priority, 1 represent then Fuel Cell priority, 2 Represent a Constant Hybrid " annotation(
        Dialog(tab = "Priority Assignment"));
      parameter Real icePriorityAssignment = 0 "0 represent the Diesel Engine Priority, 1 represent the  Alternative Fuel Priority" annotation(
        Dialog(tab = "Priority Assignment"));
      parameter Real iceChargingProrityAssignement = 0 "0 represent the Diesel Generator for charging,  1 Represent the  Alternative  Fuel generator  for charging, 2 for Fuel cell for charging" annotation(
        Dialog(tab = "Priority Assignment"));
      parameter Real dieselEngineShare = 0.5 "Fraction of P_load assigned to ICE Engine" annotation(
        Dialog(tab = "Constant Hybrid Configuration"));
      parameter Real fuelCellShare = 0.5 "Fraction of P_load assigned to Fuel Cell Engine" annotation(
        Dialog(tab = "Constant Hybrid Configuration"));
      parameter Real fuelCellPowerBandCount = 4;
      //Real powerBandSize;
      Boolean chargingStatus "Battery Charging Required ?";
      parameter Real IceChargingSOCmax = 0.7 "Max Battery SOC  to charge battery with ICE engines" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      parameter Real IceChargingSOCmin = 0.1 "Min Battery SOC to charge battery with ICE engines" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      parameter Real IceTankChargingLimit = 0.3 "ICE Fuel Tank minimal SOC limit to charge the battery" annotation(
        Dialog(tab = "ICE Charging Configuration"));
      // Modelica.Blocks.Interfaces.RealOutput P_iceCharging  annotation(
      //Placement(transformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealInput dieselOptimUpperBound annotation(
        Placement(transformation(origin = {-110, -10}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput dieselOptimLowerBound annotation(
        Placement(transformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-10, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput dieselPwrAtMinBSFC annotation(
        Placement(transformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-92, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput altFuelOptimUpperBound annotation(
        Placement(transformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealInput altFuelOptimLowerBound annotation(
        Placement(transformation(origin = {-30, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealInput altFuelPwrAtMinBSFC annotation(
        Placement(transformation(origin = {-70, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}})));
    equation
      smooth_soc_max = (1 - cubicStep((1 + (SOC_1 - SOC_max)*(1/smooth_charge_percentage))));
      smooth_soc_min = cubicStep((SOC_1 - SOC_min)*(1/smooth_charge_percentage));
      
      if priorityAssignement == 0 then
        if icePriorityAssignment == 0 then
          if iceChargingProrityAssignement == 0 then              
             /*====0,0,0====*/
            if (DieselGeneratorCount*dieselOptimLowerBound < P_net and P_net < DieselGeneratorCount*dieselOptimUpperBound) then
              /*Leaf Node 1 */
              // Prioritise  Diesel, Diesel Charging , Demand within the BSFC Sweet Spot Zone
              P_dieselGen = P_net;
              // Diesel  handels all demand as  long as the demand is within thesweet spot zone
              P_altFuelGen = 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 1;
            elseif (P_net < DieselGeneratorCount*dieselOptimLowerBound and P_net > P_idling_diesel_gen) then
              /*Leaf Node 2 */
              // Prioritise  Diesel, Diesel Charging , Demand Smaller than BSFC Lower Bound  == Load Balancing Charging  ==
              P_dieselGen = if (P_rated_battery < 1) then P_net elseif (SOC_1 < 0.9 and P_rated_battery > 1) then DieselGeneratorCount*dieselOptimLowerBound else P_net;
              // if there is no battery then handels the  demand else run at  lower bound to cover ab it of charging
              P_altFuelGen = 0;
              // no need to run the alt gen with have one or not
              P_battery_1 = 0;
              // no need to run the battery
              P_fuelCellGen = 0;
              IceGenChargingRate = if (SOC_1 < 0.85 and P_rated_battery > 1) then DieselGeneratorCount*dieselOptimLowerBound - P_net else 0;
              // if there is battery and there and  then charging commence, excess energy flows into battey charging
              nodeTracker = 2;
            //elseif the demand is smaller than the idling power
            elseif (P_net < P_idling_diesel_gen or P_net == P_idling_diesel_gen) then
              /*Leaf Node Special 2.5 */
              
              P_dieselGen = if (SOC_1 < 0.85 and P_rated_battery > 1) then P_idling_diesel_gen else P_idling_diesel_gen;
              P_altFuelGen = 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = if (SOC_1 < 0.85 and P_rated_battery > 1) then P_idling_diesel_gen- P_net else 0;
              
      
              
                  
             nodeTracker = 2.5;
            elseif (P_net > DieselGeneratorCount*dieselOptimUpperBound) then
              if (SOC_1 > 0.25 and P_rated_battery > 1) then
                if (P_net > P_rated_diesel_gen) then
                  if (P_net - DieselGeneratorCount*dieselOptimUpperBound < P_rated_battery) then
                    /*Leaf Node 3 */
                    // Prioritise  Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can Cover the Gap
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                    // diesle generator runs  at upper Bound
                    /*Load Leveling*/
                    P_altFuelGen = 0;
                // alt gen not need to take part
                    P_battery_1 = (P_net - DieselGeneratorCount*dieselOptimUpperBound)*smooth_soc_min;
                // unmet power handles byt battery
                    IceGenChargingRate = 0;
                 // no charging action
                    P_fuelCellGen = 0;
                    nodeTracker = 3;
                  else
    //(P_net - DieselGeneratorCount*dieselOptimUpperBound > P_rated_battery)
                    if (P_rated_alt_gen > 0) then
    /*Leaf Node 4 */
    // Prioritise Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Exisit
                      P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
    //diesel generator  runs at upper  Bound
                      P_battery_1 = P_rated_battery*smooth_soc_min;
    // battery runs at thier max
                      P_altFuelGen = if P_net - DieselGeneratorCount*dieselOptimUpperBound - P_battery_1 > 0 then P_net - DieselGeneratorCount*dieselOptimUpperBound - P_battery_1 else 0;
    // if there is still unmet  power then handles but alt gen, if not alt gen don't run
                      IceGenChargingRate = 0;
    // no charing action
                      P_fuelCellGen = 0;
                      nodeTracker = 4;
                    else
    /*Leaf Node 5 */
    // Prioritise Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Non - Exisit
    /*RARE CASE*/
                      P_dieselGen = P_rated_diesel_gen;
    //diesel gen runs as much as possible
                      P_battery_1 = P_rated_battery*smooth_soc_min;
    //battery gen runs as much as possible
                      P_altFuelGen = 0;
    //there is no alt gen
                      IceGenChargingRate = 0;
    // no charging action
                      P_fuelCellGen = 0;
                      nodeTracker = 5;
                    end if;
                  end if;
                elseif (P_net < P_rated_diesel_gen and P_net > DieselGeneratorCount*dieselOptimUpperBound) then
                  if (P_rated_battery > P_net - DieselGeneratorCount*dieselOptimUpperBound) then
                    /*Leaf Node 6*/
                    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound  and  max Rated, Battery Can cover the gap
                    /*Load Leveling */
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                // Diesel Generator make Upper Bound
                    P_battery_1 = (P_net - DieselGeneratorCount*dieselOptimUpperBound)*smooth_soc_min;
    // battery  fill the unmet power
                    P_altFuelGen = 0;
    // no involvoment as the battery cover the demand.
                    IceGenChargingRate = 0;
    //no charging action
                    P_fuelCellGen = 0;
                    nodeTracker = 6;
                  else
    /*Leaf Node 7*/
    // /*Need further detail on allocating the unmet power  */
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
    // diesel generator runs at upper bound
                    P_battery_1 = P_rated_battery*smooth_soc_min;
    // battery works at hard as possible
                    P_altFuelGen = if (P_rated_alt_gen > 0) then if P_net - P_rated_battery - P_dieselGen > 0 then P_net - P_rated_battery - P_dieselGen else 0 else 0;
    // if there is alt gen then alt gen handles the unmet power
                    IceGenChargingRate = 0;
    // no charging action
                    P_fuelCellGen = 0;
                    nodeTracker = 7;
                  end if;
                else
    /*Default Node 1*/
    // When demand is  neither  larger than rated power not within higher bound and  rated power
                  P_dieselGen = 130001;
                  P_battery_1 = 130001;
                  P_altFuelGen = 130001;
                  IceGenChargingRate = 0;
                  P_fuelCellGen = 0;
                  nodeTracker = 0.1;
                end if;
              elseif (SOC_1 < 0.25 or P_rated_battery < 1) then
    /*Leaf Node 8*/
    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery NON - Exist.
                P_dieselGen = if (P_rated_alt_gen > 0) then DieselGeneratorCount*dieselOptimUpperBound else P_rated_diesel_gen;
    // if there is alt gen then runn at upper bound else run as hard as possible
                P_battery_1 = 0;
                P_altFuelGen = if (P_rated_alt_gen > 0) then P_net - P_dieselGen else 0;
    // if there is  altgen then fill the unmet  else it is 0
                IceGenChargingRate = 0;
                P_fuelCellGen = 0;
                nodeTracker = 8;
              else
    /*Default Node 2*/
    // When the battery is neither exist or non exist, having charge or not having charge
                P_dieselGen = 140001;
                P_altFuelGen = 140001;
                P_battery_1 = 101*smooth_soc_min;
                P_fuelCellGen = 0;
                IceGenChargingRate = 0;
                nodeTracker = 0.2;
              end if;
            else
    /*Default  Node 3 */
    // When the demand is neither with the sweet spot or, higher than the sweet  spot, or lower  than the sweet spot
              P_dieselGen = 100001;
              P_altFuelGen = 100001;
              P_battery_1 = 101*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 0.3;
            end if;
          elseif iceChargingProrityAssignement == 1 then
    /*====0,0,1====*/
            if (DieselGeneratorCount*dieselOptimLowerBound < P_net and P_net < DieselGeneratorCount*dieselOptimUpperBound) then
    /*Leaf Node 9 */
    // Prioritise  Diesel, Alt Fuel Charging , Demand within the BSFC Sweet Spot Zone
              P_dieselGen = P_net;
    // diesel generator handels the deamnd
              P_altFuelGen = if (P_rated_alt_gen > 0) then if (SOC_1 < 0.9 and P_rated_battery > 1) then AlthFuelGenChargingRate else 0 else 0;
    // if there is a  altFuelGen and the battery need s charging then  charge the battery
              P_battery_1 = 0;
    // no battery need s to be invovled
              P_fuelCellGen = 0;
              IceGenChargingRate = P_altFuelGen;
    // charing action handels by the alt gen
              nodeTracker = 9;
            elseif (P_net < DieselGeneratorCount*dieselOptimLowerBound and P_net > P_idling_diesel_gen) then
    /*Leaf Node 10 */
    // Prioritise  Diesel, Alt Fuel Charging , Demand Smaller than BSFC Lower Bound  == Load Balancing then Charging  ==
              P_dieselGen = P_net;
    // the diesel generator handels the demand
              P_altFuelGen = if (P_rated_alt_gen > 0) then if (SOC_1 < 0.9 and P_rated_battery > 1) then AlthFuelGenChargingRate else 0 else 0;
    // if there is a  altFuelGen and the battery need s charging then  charge the battery
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = P_altFuelGen;
    // charing action handels by the alt gen
              nodeTracker = 10;
    // elseif the demand is smaller the idling power
            elseif (P_net < P_idling_diesel_gen or P_net == P_idling_diesel_gen) then
    /*Leaf Node Special 10.5 */
              P_dieselGen = P_idling_diesel_gen;
              P_altFuelGen = if (P_rated_alt_gen > 0) then if (SOC_1 < 0.9 and P_rated_battery > 1) then AlthFuelGenChargingRate else 0 else 0;
    // if there is a  altFuelGen and the battery need s charging then  charge the battery
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = P_altFuelGen;
              nodeTracker = 10.5;
            elseif (P_net > DieselGeneratorCount*dieselOptimUpperBound) then
              if (SOC_1 > 0.25 and P_rated_battery > 1) then
                if (P_net > P_rated_diesel_gen) then
                  if (P_net - DieselGeneratorCount*dieselOptimUpperBound < P_rated_battery) then
    /*Leaf Node 11 */
    // Prioritise  Diesel, Alt Fuel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can Cover the Gap
    /*Load Leveling */
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
    // diesel generator runs at upper bound
                    P_battery_1 = (P_net - P_dieselGen)*smooth_soc_min;
    // battery handele the unmet
                    P_altFuelGen = if (P_rated_alt_gen > 0) then if (SOC_1 < 0.9 and P_rated_battery > 1) then AlthFuelGenChargingRate else 0 else 0;
    // alt gen charge the battery if there is one
                    IceGenChargingRate = P_altFuelGen;
    // charing actions handles by alt gen
                    P_fuelCellGen = 0;
                    nodeTracker = 11;
                  else
    //(P_net - DieselGeneratorCount*dieselOptimUpperBound > P_rated_battery)
                    if (P_rated_alt_gen > 0) then
    /*Leaf Node 12 */
    // Prioritise Diesel,  Alt Fuel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Exisit
                      P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
    // Diesle gen runs at upper bound
                      P_battery_1 = P_rated_battery*smooth_soc_min;
    // battery runs as much as possible
                      P_altFuelGen = if P_net - DieselGeneratorCount*dieselOptimUpperBound - P_battery_1 > 0 then P_net - DieselGeneratorCount*dieselOptimUpperBound - P_battery_1 else 0;
    // if there is still unmet power  , then handles by alt gen
                      IceGenChargingRate = 0;
    // no charging action
                      P_fuelCellGen = 0;
                      nodeTracker = 12;
                    else
    /*Leaf Node 13 */
    // Prioritise Diesel, Alt Fuel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Non - Exisit
    /*RARE CASE*/
                      P_dieselGen = P_rated_diesel_gen;
    // diesel generator runs as hard as possible
                      P_battery_1 = P_rated_battery*smooth_soc_min;
    // battery run as hard as possible
                      P_altFuelGen = 0;
    // no alt gen
                      IceGenChargingRate = 0;
                      P_fuelCellGen = 0;
                      nodeTracker = 13;
                    end if;
                  end if;
                elseif (P_net < P_rated_diesel_gen and P_net > DieselGeneratorCount*dieselOptimUpperBound) then
                  if (P_rated_battery > P_net - DieselGeneratorCount*dieselOptimUpperBound) then
    /*Leaf Node 14*/
    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound  and  max Rated, Battery Can cover the gap
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
    // diesel  generator runs to upper bounds
                    P_battery_1 = (P_net - DieselGeneratorCount*dieselOptimUpperBound)*smooth_soc_min;
    // the battery covers the unmet
                    P_altFuelGen = if (P_rated_alt_gen > 0) then if (SOC_1 < 0.9 and P_rated_battery > 1) then AlthFuelGenChargingRate else 0 else 0;
    // if there is alt gen then if the battery needs to be chares  alt gen charging,
                    IceGenChargingRate = P_altFuelGen;
                    P_fuelCellGen = 0;
                    nodeTracker = 14;
                  else
    /*Leaf Node 15*/
    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound and max Rated, Battery Can Not cover the gap
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
    // diesel generator runs to upper bounds
                    P_battery_1 = P_rated_battery*smooth_soc_min;
    // diesel generator runs as hard as possible
                    P_altFuelGen = if (P_rated_alt_gen > 0) then P_net - P_rated_battery - P_dieselGen else 0;
    // if there  is the alt gen, the alt gen runs at
                    IceGenChargingRate = 0;
                    P_fuelCellGen = 0;
                    nodeTracker = 15;
                  end if;
                else
    /*Default Node 4*/
    // When demand is  neither  larger than rated power not within higher bound and  rated power
                  P_dieselGen = 230001;
                  P_battery_1 = 230001;
                  P_altFuelGen = 230001;
                  IceGenChargingRate = 0;
                  P_fuelCellGen = 0;
                  nodeTracker = 0.4;
                end if;
              elseif (SOC_1 < 0.25 or P_rated_battery < 1) then
    /*Leaf Node 16*/
    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery NON - Exist or unable to provide charge .
    /*Need Further refinement */
                P_dieselGen = if (P_rated_alt_gen > 0) then DieselGeneratorCount*dieselOptimUpperBound else P_rated_diesel_gen;
    //if there is alt gen then die gen runs at upper bound or else runs at max diesel gen
                P_battery_1 = 0;
    // no battery  power
                P_altFuelGen = if (P_rated_alt_gen > 0) then if (P_rated_battery < 1) then P_net - P_dieselGen else (P_net - P_dieselGen) + AlthFuelGenChargingRate else 0;
    // if there ius alt gen then check if ther eis battery, if there is battery run at  unmet + RatedChargerPower  else
                IceGenChargingRate = if (P_rated_alt_gen > 0) then if (P_rated_battery < 0) then 0 else (AlthFuelGenChargingRate) else 0;
    // if there is alt gen , then checl if there is a attery, if there is  then  charging at  AlthFuelGenChargingRate else  0
                P_fuelCellGen = 0;
                nodeTracker = 16;
              else
    /*Default Node 5*/
    // When the battery is neither exist or non exist, having charge or not having charge
                P_dieselGen = 240001;
                P_altFuelGen = 240001;
                P_battery_1 = 101*smooth_soc_min;
                P_fuelCellGen = 0;
                IceGenChargingRate = 0;
                nodeTracker = 0.5;
              end if;
            else
    /*Default  Node 6 */
    // When the demand is neither with the sweet spot or, higher than the sweet  spot, or lower  than the sweet spot
              P_dieselGen = 20001;
              P_altFuelGen = 200001;
              P_battery_1 = 101*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 0.6;
            end if;
          end if;
        elseif icePriorityAssignment == 1 then
          if iceChargingProrityAssignement == 0 then
    /*====0,1,0====*/
            if (DieselGeneratorCount*dieselOptimLowerBound < P_net and P_net < AltFuelGeneratorCount*altFuelOptimUpperBound) then
    /*Leaf Node 17 */
    // Prioritise Alt Fuel, Diesel Charging , Demand within the BSFC Sweet Spot Zone
              P_altFuelGen = P_net;
              P_dieselGen = if (P_rated_diesel_gen > 0) then if (SOC_1 < 0.9 and P_rated_battery > 1) then DieselGenChargingRate else 0 else 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = P_dieselGen;
              nodeTracker = 17;
            elseif (P_net < AltFuelGeneratorCount*altFuelOptimLowerBound and P_net > P_idling_alt_gen) then
    /*Leaf Node 18 */
    // Prioritise Alt Fuel, Diesel Charging , Demand Smaller than BSFC Lower Bound  == Load Balancing then Charging  ==
              P_altFuelGen = P_net;
              P_dieselGen = if (P_rated_diesel_gen > 0) then if (SOC_1 < 0.9 and P_rated_battery > 1) then DieselGenChargingRate else 0 else 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = P_dieselGen;
              nodeTracker = 18;
    //elseif the demand is smaller the idling power
            elseif (P_net < P_idling_alt_gen or P_net == P_idling_diesel_gen) then
              P_altFuelGen = P_idling_alt_gen;
              P_dieselGen = 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = if (P_rated_diesel_gen > 0) then if (SOC_1 < 0.9 and P_rated_battery > 1) then DieselGenChargingRate else 0 else 0;
              nodeTracker = 18.25;
            elseif (P_net > AltFuelGeneratorCount*altFuelOptimUpperBound) then
              if (SOC_1 > 0.25 and P_rated_battery > 1) then
                if (P_net > P_rated_alt_gen) then
                  if (P_net - DieselGeneratorCount*dieselOptimUpperBound < P_rated_battery) then
    /*Leaf Node 19 */
    // Prioritise Alt Fuel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can Cover the Gap
                    P_altFuelGen = AltFuelGeneratorCount*altFuelOptimUpperBound;
                    P_battery_1 = (P_net - P_altFuelGen)*smooth_soc_min;
                    P_dieselGen = if (P_rated_diesel_gen > 0) then if SOC_1 < 0.9 then DieselGenChargingRate else 0 else 0;
                    IceGenChargingRate = P_dieselGen;
                    P_fuelCellGen = 0;
                    nodeTracker = 19;
                  else
    //(P_net - DieselGeneratorCount*dieselOptimUpperBound > P_rated_battery)
                    if (P_dieselGen > 0) then
    /*Leaf Node 20 */
    // Prioritise Alt Fuel,  Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Exisit
                      P_altFuelGen = AltFuelGeneratorCount*altFuelOptimUpperBound;
                      P_battery_1 = P_rated_battery*smooth_soc_min;
                      P_dieselGen = if P_net - P_altFuelGen - P_battery_1 > 0 then P_net - P_altFuelGen - P_battery_1 else 0;
                      IceGenChargingRate = 0;
                      P_fuelCellGen = 0;
                      nodeTracker = 20;
                    else
    /*Leaf Node 21 */
    // Prioritise Alt Fuel,Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Non - Exisit
    /*RARE CASE*/
                      P_altFuelGen = P_rated_alt_gen;
                      P_battery_1 = P_rated_battery*smooth_soc_min;
                      P_dieselGen = 0;
                      IceGenChargingRate = 0;
                      P_fuelCellGen = 0;
                      nodeTracker = 21;
                    end if;
                  end if;
                elseif (P_net < P_rated_alt_gen and P_net > AltFuelGeneratorCount*altFuelOptimUpperBound) then
                  if (P_rated_battery > P_net - AltFuelGeneratorCount*altFuelOptimUpperBound) then
    /*Leaf Node 22*/
    //Prioritise Alt Fuel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound  and  max Rated, Battery Can cover the gap
                    P_altFuelGen = AltFuelGeneratorCount*altFuelOptimUpperBound;
                    P_battery_1 = (P_net - P_altFuelGen)*smooth_soc_min;
                    P_dieselGen = if P_rated_diesel_gen > 0 then if SOC_1 < 0.9 then DieselGenChargingRate else 0 else 0;
                    IceGenChargingRate = P_dieselGen;
                    P_fuelCellGen = 0;
                    nodeTracker = 22;
                  else
    /*Leaf Node 23*/
    //Prioritise  Alt Fuel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound and max Rated, Battery Can Not cover the gap
                    P_altFuelGen = AltFuelGeneratorCount*altFuelOptimUpperBound;
                    P_battery_1 = P_rated_battery*smooth_soc_min;
                    P_dieselGen = if P_rated_diesel_gen > 0 then P_net - P_altFuelGen - P_battery_1 else 0;
                    IceGenChargingRate = P_dieselGen;
                    P_fuelCellGen = 0;
                    nodeTracker = 23;
                  end if;
                else
    /*Default Node 7*/
    // When demand is  neither  larger than rated power not within higher bound and  rated power
                  P_dieselGen = 330001;
                  P_battery_1 = 301*smooth_soc_min;
                  P_altFuelGen = 330001;
                  IceGenChargingRate = 0;
                  P_fuelCellGen = 0;
                  nodeTracker = 0.7;
                end if;
              elseif (SOC_1 < 0.25 or P_rated_battery < 1) then
    /*Leaf Node 24*/
    //Prioritise Alt Fuel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery NON - Exist.
                P_altFuelGen = if (P_rated_diesel_gen > 0) then AltFuelGeneratorCount*altFuelOptimUpperBound else P_rated_alt_gen;
                P_battery_1 = 0;
                P_dieselGen = if (P_rated_diesel_gen > 0) then if (P_rated_battery < 1) then 0 else AlthFuelGenChargingRate else 0;
                IceGenChargingRate = P_altFuelGen;
                P_fuelCellGen = 0;
                nodeTracker = 24;
              else
    /*Default Node 8*/
    // When the battery is neither exist or non exist, having charge or not having charge
                P_dieselGen = 340001;
                P_altFuelGen = 340001;
                P_battery_1 = 301*smooth_soc_min;
                P_fuelCellGen = 0;
                IceGenChargingRate = 0;
                nodeTracker = 0.8;
              end if;
            else
    /*Default  Node 9  */
    // When the demand is neither with the sweet spot or, higher than the sweet  spot, or lower  than the sweet spot
              P_dieselGen = 30001;
              P_altFuelGen = 300001;
              P_battery_1 = 301*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 0.9;
            end if;
          elseif iceChargingProrityAssignement == 1 then
    /*====0,1,1====*/
            if (AltFuelGeneratorCount*altFuelOptimLowerBound < P_net and P_net < AltFuelGeneratorCount*altFuelOptimLowerBound) then
    /*Leaf Node 25 */
    // Prioritise Alt Fuel, Alt Fuel Charging , Demand within the BSFC Sweet Spot Zone
              P_altFuelGen = P_net;
              P_dieselGen = 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 25;
            elseif (P_net < AltFuelGeneratorCount*altFuelOptimLowerBound and P_net > P_idling_alt_gen) then
    /*Leaf Node 26 */
    // Prioritise Alt Fuel, Diesel Charging , Demand Smaller than BSFC Lower Bound  == Load Balancing then Charging  ==
              P_altFuelGen = if (SOC_1 > 0.25 and P_rated_battery > 1) then AltFuelGeneratorCount*altFuelOptimLowerBound else P_net;
              P_dieselGen = 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = if (SOC_1 > 0.25 and P_rated_battery > 1) then AltFuelGeneratorCount*altFuelOptimLowerBound - P_net else 0;
              nodeTracker = 26;
    // elseif the demand is smaller the idling power
            elseif (P_net < P_idling_alt_gen or P_net == P_idling_diesel_gen) then
              P_altFuelGen = P_idling_alt_gen;
              P_dieselGen = 0;
              P_battery_1 = 0;
              P_fuelCellGen = 0;
              IceGenChargingRate = if (SOC_1 > 0.25 and P_rated_battery > 1) then P_idling_alt_gen - P_net else 0;
              nodeTracker = 26.5;
            elseif (P_net > AltFuelGeneratorCount*altFuelOptimUpperBound) then
              if (SOC_1 > 0.25 and P_rated_battery > 1) then
                if (P_net > P_rated_diesel_gen) then
                  if (P_net - AltFuelGeneratorCount*altFuelOptimUpperBound < P_rated_battery) then
    /*Leaf Node 27 */
    // Prioritise Alt Fuel,  Alt Fuel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can Cover the Gap
                    P_altFuelGen = AltFuelGeneratorCount*altFuelOptimUpperBound;
                    P_battery_1 = (P_net - P_altFuelGen)*smooth_soc_min;
                    P_dieselGen = 0;
                    IceGenChargingRate = 0;
                    P_fuelCellGen = 0;
                    nodeTracker = 27;
                  else
    //(P_net - DieselGeneratorCount*dieselOptimUpperBound > P_rated_battery)
                    if (P_dieselGen > 0) then
    /*Leaf Node 28 */
    // Prioritise Alt Fuel,  Alt Fuel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Diesel Gen Exist
                      P_altFuelGen = AltFuelGeneratorCount*altFuelOptimUpperBound;
                      P_battery_1 = P_rated_battery*smooth_soc_min;
                      P_dieselGen = if P_net - P_altFuelGen - P_battery_1 > 0 then P_net - P_altFuelGen - P_battery_1 else 0;
                      IceGenChargingRate = 0;
                      P_fuelCellGen = 0;
                      nodeTracker = 28;
                    else
    /*Leaf Node 29 */
    // Prioritise Alt Fuel,Alt Fuel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Diesel Gen Non - Exist
    /*RARE CASE*/
                      P_altFuelGen = P_rated_alt_gen;
                      P_battery_1 = P_rated_battery*smooth_soc_min;
                      P_dieselGen = 0;
                      IceGenChargingRate = 0;
                      P_fuelCellGen = 0;
                      nodeTracker = 29;
                    end if;
                  end if;
                elseif (P_net < P_rated_alt_gen and P_net > AltFuelGeneratorCount*altFuelOptimUpperBound) then
                  if (P_rated_battery > P_net - AltFuelGeneratorCount*altFuelOptimUpperBound) then
    /*Leaf Node 30*/
    //Prioritise Alt Fuel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound  and  max Rated, Battery Can cover the gap
                    P_altFuelGen = AltFuelGeneratorCount*altFuelOptimUpperBound;
                    P_battery_1 = (P_net - P_altFuelGen)*smooth_soc_min;
                    P_dieselGen = 0;
                    IceGenChargingRate = P_dieselGen;
                    P_fuelCellGen = 0;
                    nodeTracker = 30;
                  else
    /*Leaf Node 31*/
    //Prioritise  Alt Fuel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN       BETWEEN Higher Bound and max Rated, Battery Can Not cover the gap
                    P_altFuelGen = AltFuelGeneratorCount*altFuelOptimUpperBound;
                    P_battery_1 = P_rated_battery*smooth_soc_min;
                    P_dieselGen = if P_rated_diesel_gen > 0 then P_net - P_altFuelGen - P_battery_1 else 0;
                    IceGenChargingRate = P_dieselGen;
                    P_fuelCellGen = 0;
                    nodeTracker = 31;
                  end if;
                else
    /*Default Node 10*/
    // When demand is  neither  larger than rated power not within higher bound and  rated power
                  P_dieselGen = 430001;
                  P_battery_1 = 401*smooth_soc_min;
                  P_altFuelGen = 430001;
                  IceGenChargingRate = 0;
                  P_fuelCellGen = 0;
                  nodeTracker = 1.1;
                end if;
              elseif (SOC_1 < 0.25 or P_rated_battery < 1) then
    /*Leaf Node 32*/
    //Prioritise Alt Fuel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery NON - Exist.
                P_altFuelGen = if (P_rated_diesel_gen > 0) then AltFuelGeneratorCount*altFuelOptimUpperBound else P_rated_alt_gen;
                P_battery_1 = 0;
                P_dieselGen = if (P_rated_diesel_gen > 0) then if (P_rated_battery < 1) then 0 else AlthFuelGenChargingRate else 0;
                IceGenChargingRate = P_altFuelGen;
                P_fuelCellGen = 0;
                nodeTracker = 32;
              else
    /*Default Node 11*/
    // When the battery is neither exist or non exist, having charge or not having charge
                P_dieselGen = 440001;
                P_altFuelGen = 440001;
                P_battery_1 = 401*smooth_soc_min;
                P_fuelCellGen = 0;
                IceGenChargingRate = 0;
                nodeTracker = 1.2;
              end if;
            else
    /*Default  Node 12  */
    // When the demand is neither with the sweet spot or, higher than the sweet  spot, or lower  than the sweet spot
              P_dieselGen = 40001;
              P_altFuelGen = 400001;
              P_battery_1 = 401*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 1.3;
            end if;
          end if;
        end if;
      elseif priorityAssignement == 1 then
        if icePriorityAssignment == 0 then
          if iceChargingProrityAssignement == 0 then
            if (DieselGeneratorCount*dieselOptimLowerBound < P_net and P_net < DieselGeneratorCount*dieselOptimUpperBound) then
              P_dieselGen = P_net;
              P_altFuelGen = 0;
              P_battery_1 = 100*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            elseif (P_net < DieselGeneratorCount*dieselOptimLowerBound) then
              P_dieselGen = if (P_rated_battery == 0) then P_net elseif (SOC_1 < 0.9 and P_rated_battery > 0) then DieselGeneratorCount*dieselOptimLowerBound else P_net;
              P_altFuelGen = 0;
              P_battery_1 = 100*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = if (SOC_1 < 0.9 and P_rated_battery > 0) then DieselGeneratorCount*dieselOptimLowerBound - P_net else 0;
              nodeTracker = 2.9;
            elseif (P_net > DieselGeneratorCount*dieselOptimUpperBound) then
              if (SOC_1 > 0.25 and P_rated_battery > 0) then
                if (P_net > P_rated_diesel_gen) then
                  if (P_net - DieselGeneratorCount*dieselOptimUpperBound < P_rated_battery) then
    /*Leaf Node 3 */
    // Prioritise  Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can Cover the Gap
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                    P_altFuelGen = 0;
                    P_battery_1 = (P_net - DieselGeneratorCount*dieselOptimLowerBound)*smooth_soc_min;
                    IceGenChargingRate = 0;
                    P_fuelCellGen = 0;
                    nodeTracker = 2.9;
                  else
    //(P_net - DieselGeneratorCount*dieselOptimUpperBound > P_rated_battery)
                    if (P_rated_alt_gen > 0) then
    /*Leaf Node 4 */
    // Prioritise Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Exisit
                      P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                      P_battery_1 = P_rated_battery*smooth_soc_min;
                      P_altFuelGen = P_net - DieselGeneratorCount*dieselOptimUpperBound - P_battery_1;
                      IceGenChargingRate = 0;
                      P_fuelCellGen = 0;
                      nodeTracker = 2.9;
                    else
    /*Leaf Node 5 */
    // Prioritise Diesel, Diesel Charging, Demand Bigger than BSFC Higher Bound, Battery Exist, Demand Also Higher than Max Rated Power, Battery Can NOT Cover the Gap, Alt Gen Non - Exisit
    /*RARE CASE*/
                      P_dieselGen = P_rated_diesel_gen;
                      P_battery_1 = P_rated_battery*smooth_soc_min;
                      P_altFuelGen = 0;
                      IceGenChargingRate = 0;
                      P_fuelCellGen = 0;
                      nodeTracker = 2.9;
                    end if;
                  end if;
                elseif (P_net < P_rated_diesel_gen and P_net > DieselGeneratorCount*dieselOptimUpperBound) then
                  if (P_rated_battery > P_net - DieselGeneratorCount*dieselOptimUpperBound) then
    /*Leaf Node 6*/
    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound  and  max Rated, Battery Can cover the gap
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                    P_battery_1 = (P_net - DieselGeneratorCount*dieselOptimUpperBound)*smooth_soc_min;
                    P_altFuelGen = 0;
                    IceGenChargingRate = 0;
                    P_fuelCellGen = 0;
                    nodeTracker = 2.9;
                  else
    /*Leaf Node 7*/
    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery Exist, Demand IN BETWEEN Higher Bound and max Rated, Battery Can Not cover the gap
                    P_dieselGen = DieselGeneratorCount*dieselOptimUpperBound;
                    P_battery_1 = P_rated_battery*smooth_soc_min;
                    P_altFuelGen = if (P_rated_alt_gen > 0) then P_net - P_rated_battery - P_dieselGen else 0;
                    IceGenChargingRate = 0;
                    P_fuelCellGen = 0;
                    nodeTracker = 2.9;
                  end if;
                else
    /*Default Node 1*/
    // When demand is  neither  larger than rated power not within higher bound and  rated power
                  P_dieselGen = 130001;
                  P_battery_1 = 130001;
                  P_altFuelGen = 130001;
                  IceGenChargingRate = 0;
                  P_fuelCellGen = 0;
                  nodeTracker = 2.9;
                end if;
              elseif (SOC_1 < 0.25 or P_rated_battery == 0) then
    /*Leaf Node 7*/
    //Prioritise Diesel, Diesel Charging , Demand Bigger than BSFC  Higher Bound. Battery NON - Exist.
                P_dieselGen = if (P_rated_alt_gen > 0) then DieselGeneratorCount*dieselOptimUpperBound else P_rated_diesel_gen;
                P_battery_1 = 0;
                P_altFuelGen = if (P_rated_alt_gen > 0) then P_net - P_dieselGen else 0;
                IceGenChargingRate = 0;
                P_fuelCellGen = 0;
                nodeTracker = 2.9;
              else
    /*Default Node 2*/
    // When the battery is neither exist or non exist, having charge or not having charge
                P_dieselGen = 140001;
                P_altFuelGen = 140001;
                P_battery_1 = 101*smooth_soc_min;
                P_fuelCellGen = 0;
                IceGenChargingRate = 0;
                nodeTracker = 2.9;
              end if;
            else
              P_dieselGen = 300001;
              P_altFuelGen = 300001;
              P_battery_1 = 301*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            end if;
          elseif iceChargingProrityAssignement == 1 then
            if (DieselGeneratorCount*dieselOptimLowerBound < P_net and P_net < DieselGeneratorCount*dieselOptimUpperBound) then
              P_dieselGen = 350000;
              P_altFuelGen = 350000;
              P_battery_1 = 300*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            elseif (P_net < DieselGeneratorCount*dieselOptimLowerBound) then
              P_dieselGen = 360000;
              P_altFuelGen = 360000;
              P_battery_1 = 300*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            elseif (P_net > DieselGeneratorCount*dieselOptimUpperBound) then
              P_dieselGen = 370000;
              P_altFuelGen = 320000;
              P_battery_1 = 300*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            else
              P_dieselGen = 350001;
              P_altFuelGen = 350001;
              P_battery_1 = 301*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            end if;
          end if;
        elseif icePriorityAssignment == 1 then
          if iceChargingProrityAssignement == 0 then
            if (DieselGeneratorCount*dieselOptimLowerBound < P_net and P_net < DieselGeneratorCount*dieselOptimUpperBound) then
              P_dieselGen = 400000;
              P_altFuelGen = 400000;
              P_battery_1 = 400*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            elseif (P_net < DieselGeneratorCount*dieselOptimLowerBound) then
              P_dieselGen = 410000;
              P_altFuelGen = 410000;
              P_battery_1 = 300*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            elseif (P_net > DieselGeneratorCount*dieselOptimUpperBound) then
              P_dieselGen = 420000;
              P_altFuelGen = 420000;
              P_battery_1 = 300*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            else
              P_dieselGen = 400001;
              P_altFuelGen = 400001;
              P_battery_1 = 401*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            end if;
          elseif iceChargingProrityAssignement == 1 then
            if (DieselGeneratorCount*dieselOptimLowerBound < P_net and P_net < DieselGeneratorCount*dieselOptimUpperBound) then
              P_dieselGen = 450000;
              P_altFuelGen = 450000;
              P_battery_1 = 400*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            elseif (P_net < DieselGeneratorCount*dieselOptimLowerBound) then
              P_dieselGen = 460000;
              P_altFuelGen = 460000;
              P_battery_1 = 400*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            elseif (P_net > DieselGeneratorCount*dieselOptimUpperBound) then
              P_dieselGen = 470000;
              P_altFuelGen = 470000;
              P_battery_1 = 400*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            else
              P_dieselGen = 450001;
              P_altFuelGen = 450001;
              P_battery_1 = 401*smooth_soc_min;
              P_fuelCellGen = 0;
              IceGenChargingRate = 0;
              nodeTracker = 2.9;
            end if;
    // detailed checker end
          end if;
    // charging priority end
        end if;
    // ice priority assignement end
      end if;
    // priorityAssignement end
      annotation(
        Icon(graphics = {Rectangle(origin = {4, 17}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-60, 45}, {60, -65}})}));
    end MethanolBatteryDieselControllerCore;

    /*MultiControllerCore*/

    model MultiControllerCore "Controller for a set of 2 batteries and a set of desiel generators"
      //External Function Repository for Local Use//
      /*cubicStep function*/

      function cubicStep "Cubic step function"
        input Real tau "Abcissa";
        output Real y "Value";
      algorithm
        y := if tau < 0 then 0 else (if tau > 1 then 1 else (3 - 2*tau)*tau^2);
      end cubicStep;

      /*minLocal function*/

      function minLocal
        input Real a;
        input Real b;
        output Real result;
      algorithm
        result := if a < b then a else b;
      end minLocal;

      /*maxLocal function*/

      function maxLocal
        input Real a;
        input Real b;
        output Real result;
      algorithm
        result := if a > b then a else b;
      end maxLocal;

      parameter Real SOC_max = 0.9 "Max State of Charge" annotation(
        Dialog(group = "Battery"));
      parameter Real SOC_min = 0.1 "Min State of Charge" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_charging_max = 2e5 "Max Charing Rate" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_charging_min = -P_charging_max "Maximum discharge rate" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power P_rated_diesel_gen "Generator max output" annotation(
        Dialog(group = "Diesel generator"));
      parameter Modelica.Units.SI.Power P_rated_combined_gen "Combined avalible generator output" annotation(
        Dialog(group = "All generators"));
      parameter Modelica.Units.SI.Power P_rated_alt_gen "Alternative Fuel Generator max_output " annotation(
        Dialog(group = "All generators"));
      parameter Modelica.Units.SI.Power P_ref_FC_max_rate;
      //Modelica.Units.SI.Power P_GenCharging "The acctual amount of generator power assigned to charing battery";
      // Modelica.Units.SI.Power Gen4Battery  "Generator's Contribution to Recharge the Battery";
      parameter Modelica.Units.SI.Power P_GenCharging_L = 50000 "Emergency Charing Rate in case the battery dips below SOC 0.2";
      parameter Modelica.Units.SI.Power P_GenCharging_N = 200 "Constant Charging when battery is between 0.2 to 0.7 SOC";
      parameter Modelica.Units.SI.Power P_GenCharging_S = 0 "Realx";
      parameter Real dieselP_start = 0 "Initital power by generator";
      parameter Real altFuelP_start = 0 "Initial power by alternative fuel generator";
      parameter Real batteryP_start = 0 "Initial power by battery";
      parameter Real fuelcellP_start = 0 "Initial Power of fuel cell";
      Modelica.Units.SI.Power P_net = -(P_load_1) "Power Deffciecy is (- load)";
      output Modelica.Units.SI.Power P_surplus_1 "Surplus Power for Battery 1";
      Modelica.Blocks.Interfaces.RealInput P_renew_1 "Power produced by renewable energy source 1 (photovoltaics, wind, hydro) in W" annotation(
        Placement(transformation(origin = {-50, 50}, extent = {{-70, 10}, {-50, 30}}), iconTransformation(origin = {-50, 28}, extent = {{-70, 10}, {-50, 30}})));
      Modelica.Blocks.Interfaces.RealInput P_load_1 "Load power in W" annotation(
        Placement(transformation(origin = {-220, 10}, extent = {{100, 30}, {120, 50}}), iconTransformation(origin = {-220, 48}, extent = {{100, 30}, {120, 50}})));
      Modelica.Blocks.Interfaces.RealInput SOC_1(min = SOC_min, max = SOC_max) "State of charge in p.u." annotation(
        Placement(transformation(origin = {-110, 32}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-110, -68}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput P_battery_1(start = batteryP_start) "Battery_1 charging power" annotation(
        Placement(transformation(origin = {0, 30}, extent = {{100, 30}, {120, 50}}), iconTransformation(origin = {0, 30}, extent = {{100, 30}, {120, 50}})));
      Modelica.Blocks.Interfaces.RealOutput P_dieselGen(start = dieselP_start) "Power produced by diesel generators" annotation(
        Placement(transformation(extent = {{100, -50}, {120, -30}}), iconTransformation(origin = {-40, 22}, extent = {{140, -70}, {168, -42}})));
      Modelica.Blocks.Interfaces.RealOutput P_altFuelGen(start = altFuelP_start) "Power produced by alternative fuel generators" annotation(
        Placement(transformation(origin = {110, -72}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput P_fuelCellGen(start = fuelcellP_start) annotation(
        Placement(transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, 20}, extent = {{-10, -10}, {10, 10}})));
      parameter Real smooth_charge_percentage = 0.1;
      Real smooth_soc_max;
      //Smooth the charge close to SOC_max,starting at smooth_charge_percentage from SOC_max
      Real smooth_soc_min;
      //Smooth the charge close to SOC_min,starting at smooth_charge_percentage from SOC_min
      Real previousPnet(start = 0);
      Real VarPnet;
      Real dieselcall(start = 1);
      Real fuelcellcall(start = 1);
      Modelica.Blocks.Interfaces.RealInput SOC_tank(min = 0, max = 1) "State of tank" annotation(
        Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270), iconTransformation(origin = {-28, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
      parameter Real SOC_tank_sec = 0.2 "Tank level limit to activate second stage minimal Battery SOC";
      parameter Real SOC_min_sec = 0.005 "Minimum state of charge, second stage" annotation(
        Dialog(group = "Battery"));
      parameter Modelica.Units.SI.Power MinValy annotation(
        Dialog(tab = "Battery Controls"));
      parameter Modelica.Units.SI.Power MaxValu annotation(
        Dialog(tab = "Battery Controls"));
      parameter Real BatteryDieselCont annotation(
        Dialog(tab = "Battery Controls"));
      parameter Real priorityAssignement "0 represent the Generator Priority, 1 represent then Fuel Cell priority, 2 Represent a Constant Hybrid ";
      parameter Real icePriorityAssignment "0 represent the Diesel Engine Priority, 1 represent the  Alternative Fuel Priority";
      parameter Real dieselEngineShare = 0.5 "Fraction of P_load assigned to ICE Engine";
      parameter Real fuelCellShare = 0.5 "Fraction of P_load assigned to Fuel Cell Engine";
      parameter Real fuelCellPowerBandCount = 4;
      Real powerBandSize;
    equation
      if priorityAssignement == 0 then
        smooth_soc_max = (1 - cubicStep((1 + (SOC_1 - SOC_max)*(1/smooth_charge_percentage))));
        if SOC_tank > SOC_tank_sec then
          smooth_soc_min = cubicStep((SOC_1 - SOC_min)*(1/smooth_charge_percentage));
        else
          smooth_soc_min = cubicStep((SOC_1 - SOC_min_sec)*(1/smooth_charge_percentage));
        end if;
        previousPnet = delay(P_net, 60);
        if icePriorityAssignment == 0 then
// if the ice priority is 0, which means to favor diesel generator at all tim
          if (-P_net > P_rated_diesel_gen) then
// if the power requirment is bigger than the rated diesel gen
            P_dieselGen = P_rated_diesel_gen;
// then allocate the diesel generators to run at their combined  power output
            P_altFuelGen = P_rated_combined_gen - P_rated_diesel_gen;
// allocate the the rest of the unmet  power to methanol / alternative fuel engine
          else
// if the power requirment is less than the the max of diesel gen set can produce
            P_dieselGen = -P_net;
//  then allocate all the power demand to  diesel Generators
            P_altFuelGen = 0;
// methanol engines gets none
          end if;
        elseif icePriorityAssignment == 1 then
// if the ice priority is 1, which means to favor alternative fuel generator at all time
          if (-P_net > P_rated_alt_gen) then
// if the power demand is bigger than what  alternative fuel can provide (Max rated methanol gen = Max Combined Gen  - Max  DieselGen)
            P_altFuelGen = P_rated_alt_gen;
// then allocate max rated power foralternative fuel engine
            P_dieselGen = (-P_net) - (P_rated_alt_gen);
// the rest goes the diesel generator
          else
// if the alternative generators can handle the power demand
            P_altFuelGen = -P_net;
// allocate all the power demand to the alternative generators
            P_dieselGen = 0;
// no power is allocated for the diesl generators
          end if;
        end if;
        P_battery_1 = minLocal(P_charging_max*smooth_soc_max, maxLocal(P_charging_min*smooth_soc_min, ((P_dieselGen + P_altFuelGen) - (-P_net))*(if ((P_dieselGen + P_altFuelGen) - (-P_net)) >= 0 then smooth_soc_max else smooth_soc_min)));
        powerBandSize = P_ref_FC_max_rate/fuelCellPowerBandCount;
        P_fuelCellGen = ceil((minLocal(P_ref_FC_max_rate, maxLocal(0, -P_net + P_battery_1 - (P_dieselGen + P_altFuelGen))))/P_ref_FC_max_rate)*P_ref_FC_max_rate;
        P_surplus_1 = maxLocal(0, -P_net + P_battery_1 - P_altFuelGen - P_dieselGen - P_renew_1);
      elseif priorityAssignement == 1 then
        smooth_soc_max = (1 - cubicStep((1 + (SOC_1 - SOC_max)*(1/smooth_charge_percentage))));
        if SOC_tank > SOC_tank_sec then
          smooth_soc_min = cubicStep((SOC_1 - SOC_min)*(1/smooth_charge_percentage));
        else
          smooth_soc_min = cubicStep((SOC_1 - SOC_min_sec)*(1/smooth_charge_percentage));
        end if;
        previousPnet = delay(P_net, 60);
        powerBandSize = P_ref_FC_max_rate/fuelCellPowerBandCount;
        P_fuelCellGen = ceil((-P_net)/powerBandSize)*powerBandSize;
        if icePriorityAssignment == 0 then
          if ((-P_net - P_fuelCellGen) > P_rated_diesel_gen) then
            P_dieselGen = P_rated_diesel_gen;
            P_altFuelGen = (-P_net - P_fuelCellGen) - P_rated_diesel_gen;
          else
            P_dieselGen = -P_net;
            P_altFuelGen = 0;
          end if;
        elseif icePriorityAssignment == 1 then
          if (-P_net > P_rated_alt_gen) then
            P_altFuelGen = P_rated_alt_gen;
            P_dieselGen = -P_net - (P_rated_alt_gen);
          else
            P_altFuelGen = -P_net;
            P_dieselGen = 0;
          end if;
        end if;
        P_battery_1 = minLocal(P_charging_max*smooth_soc_max, maxLocal(P_charging_min*smooth_soc_min, if ((-P_net - P_renew_1 - P_dieselGen - P_altFuelGen) <= 0) then (-P_net - P_renew_1 - P_dieselGen - P_altFuelGen)*smooth_soc_max else (-P_net - P_renew_1 - P_dieselGen - P_altFuelGen)*smooth_soc_min));
        P_surplus_1 = maxLocal(0, -P_net + P_battery_1 - P_dieselGen - P_altFuelGen - P_renew_1);
      end if;
    algorithm
      VarPnet := P_net - previousPnet;
      if VarPnet > 1 or VarPnet < -1 then
        if SOC_1 > 0.1 then
          dieselcall := minLocal(BatteryDieselCont, -P_net);
        else
          dieselcall := -P_net;
        end if;
      else
        dieselcall := -P_net;
      end if;
      fuelcellcall := -P_net - dieselcall;
      dieselcall := minLocal(P_dieselGen, maxLocal(0, dieselcall));
      fuelcellcall := minLocal(P_ref_FC_max_rate, maxLocal(0, fuelcellcall));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=NLSanalyticJacobian",
        __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
    end MultiControllerCore;

    /*TranAllocator8*/

    model TransAllocator "Allocates power demand to dispatchable units"
      parameter Integer n = size(P_max, 1) "Number of dispatchable units";
      parameter Modelica.Units.SI.Power P_max[:] = {10, 1, 5} "Maximal power of the units, in decreasing priority";
      parameter Integer n2 = 1;
      final parameter Modelica.Units.SI.Power P_max_tot = sum(P_max) "Total available power";
      Modelica.Units.SI.Power P_max_i[n] "Available power up to unit i";
      Modelica.Units.SI.Power P_unit[n] "Power of the units";
      Integer i0 "Number of required running units";
      Modelica.Blocks.Interfaces.RealInput P "Power to be allocated" annotation(
        Placement(transformation(extent = {{-120, -22}, {-80, 18}})));
      Modelica.Blocks.Interfaces.RealOutput P_out[n] "Power to the different units according to pritority list" annotation(
        Placement(transformation(extent = {{88.0, -10.0}, {108.0, 10.0}}, rotation = 0.0, origin = {0.0, 0.0})));
      Modelica.Blocks.Interfaces.RealInput SOC[n] annotation(
        Placement(transformation(extent = {{-20.0, -20.0}, {20.0, 20.0}}, origin = {-26.0, -94.0}, rotation = 90.0)));
    algorithm
// Find how many units are required to run
      i0 := 2;
      P_max_i[1] := P_max[1];
      for i in 1:n loop
        if SOC[i] > 0 then
          i0 := i;
          if i > 1 then
            P_unit[i] := P - P_unit[i - 1];
          else
            P_unit[i] := P/n2;
          end if;
          if i < n then
            P_unit[i + 1:n] := zeros(n - i);
          end if;
        else
          P_unit[i] := 0;
          if i < n then
            P_max_i[i + 1] := P_max[i + 1];
          end if;
        end if;
        P_out[i] := P_unit[i]/P_max[i];
      end for;
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid), Text(extent = {{-38, 26}, {38, -30}}, lineColor = {255, 255, 255}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, textString = "A"), Text(extent = {{-100, -110}, {100, -130}}, lineColor = {0, 0, 0}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, textString = "%name")}),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end TransAllocator;

    extends BatteryDispatchableControlBase;
    parameter Modelica.Units.SI.Power P_rate_combined_ICE_gen "Combined ICE  power output";
    parameter Modelica.Units.SI.Power P_rate_gen = sum(P_max) "Generator max Input";
    parameter Modelica.Units.SI.Power P_fuelCellMax_ref = 0 "A reference to Fuel Cell Max Power";
    parameter Real diesel_gen_charge_rate = 50000;
    parameter Real alt_gen_charge_rate = 50000;
    parameter Real diesel_gen_idling_rate = 100000;
    parameter Real alt_gen_idling_rate = 100000;
    parameter Real DieselControlforBattery;
    parameter Real ParameterAssignement = 0;
    parameter Real IcePriorityAssignement = 0;
    parameter Real IceChargingPriorityAssignment = 0;
    parameter Real battery_rated_gen = 0;
    parameter Real P_max_onboard;
    parameter Real P_alt_max_onboard;
    Modelica.Blocks.Interfaces.RealInput P_load_1 annotation(
      Placement(transformation(origin = {-120, 90}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, 90}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealInput SOC_1 annotation(
      Placement(transformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealInput SOC_Tank annotation(
      Placement(transformation(origin = {-10, 118}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 122}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput P_renew_1 annotation(
      Placement(transformation(origin = {-120, 62}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput P_charge_1 annotation(
      Placement(transformation(origin = {110, 78}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput P_discharge_1 annotation(
      Placement(transformation(origin = {110, 36}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain gain(k = -1/(P_charging_min)) annotation(
      Placement(transformation(origin = {-20, 58}, extent = {{81.7507, -21.8674}, {87.4594, -16.1587}})));
    parameter Real SOC_tank_sec;
    parameter Real SOC_min_sec;
    Modelica.Blocks.Interfaces.RealInput TankmultipleSOC[n] annotation(
      Placement(transformation(origin = {6, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {20, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput AltFuelTankSOC[alt_n] annotation(
      Placement(transformation(origin = {66, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {74, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
    TransAllocator transAllocat(P_max = P_max, n2 = acutal_diesel_n) annotation(
      Placement(transformation(origin = {-40, -38}, extent = {{35.7202, -28.2798}, {64.2798, 0.279815}})));
    TransAllocator alt_Gen_Allocator(P_max = P_max_alt, n2 = actual_altFuel_n) annotation(
      Placement(transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput ControllerFC_OutPut annotation(
      Placement(transformation(origin = {-36, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {50, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput dieselUpperBound annotation(
      Placement(transformation(origin = {-109, -63}, extent = {{-9, -9}, {9, 9}}), iconTransformation(origin = {-106, -66}, extent = {{-6, -6}, {6, 6}})));
    Modelica.Blocks.Interfaces.RealInput dieselLowerBound annotation(
      Placement(transformation(origin = {-109, -79}, extent = {{-9, -9}, {9, 9}}), iconTransformation(origin = {-106, -80}, extent = {{-6, -6}, {6, 6}})));
    Modelica.Blocks.Interfaces.RealInput dieselMinimalBSFC annotation(
      Placement(transformation(origin = {-109, -93}, extent = {{-9, -9}, {9, 9}}), iconTransformation(origin = {-106, -94}, extent = {{-6, -6}, {6, 6}})));
    Modelica.Blocks.Interfaces.RealInput altFuelUpperBound annotation(
      Placement(transformation(origin = {-112, -12}, extent = {{-12, -12}, {12, 12}}), iconTransformation(origin = {-107, -17}, extent = {{-7, -7}, {7, 7}})));
    Modelica.Blocks.Interfaces.RealInput altFuelLowerBound annotation(
      Placement(transformation(origin = {-112, -28}, extent = {{-12, -12}, {12, 12}}), iconTransformation(origin = {-107, -35}, extent = {{-7, -7}, {7, 7}})));
    Modelica.Blocks.Interfaces.RealInput altFuelMinBSFC annotation(
      Placement(transformation(origin = {-112, -46}, extent = {{-12, -12}, {12, 12}}), iconTransformation(origin = {-107, -51}, extent = {{-7, -7}, {7, 7}})));
    Modelica.Blocks.Sources.RealExpression IceChargingPower(y = methanolBatteryDieselDebug.IceGenChargingRate*methanolBatteryDieselDebug.smooth_soc_max) annotation(
      Placement(transformation(origin = {32, 94}, extent = {{-8, -8}, {8, 8}})));
    MasterControllerSingleBattery.MethanolBatteryDieselDebug methanolBatteryDieselDebug(
              DieselGeneratorCount = acutal_diesel_n, 
              AltFuelGeneratorCount = actual_altFuel_n, 
              SOC_max = SOC_max, SOC_min = SOC_min, 
              DieselGenChargingRate = diesel_gen_charge_rate, 
              AlthFuelGenChargingRate = alt_gen_charge_rate, 
              P_charging_max = P_charging_max, 
              P_rated_diesel_gen = P_max_onboard, 
              P_ref_FC_max_rate = P_fuelCellMax_ref, 
              priorityAssignement = ParameterAssignement, 
              icePriorityAssignment = IcePriorityAssignement, 
              iceChargingProrityAssignement = IceChargingPriorityAssignment, 
              P_rated_combined_gen = P_rate_combined_ICE_gen, 
              P_rated_alt_gen = P_alt_max_onboard, 
              P_rated_battery = battery_rated_gen, 
              P_idling_diesel_gen = diesel_gen_idling_rate, 
              P_idling_alt_gen = alt_gen_idling_rate) annotation(
      Placement(transformation(origin = {-33, 33}, extent = {{-27, -27}, {27, 27}})));
    Modelica.Blocks.Math.Gain gain1(k = 1/(P_charging_max)) annotation(
      Placement(transformation(origin = {-8, 102}, extent = {{81.7507, -21.8674}, {87.4594, -16.1587}})));
  equation
    connect(gain.y, P_discharge_1) annotation(
      Line(points = {{68, 39}, {89, 39}, {89, 36}, {110, 36}}, color = {0, 0, 127}));
    connect(transAllocat.P_out, P_dispatch) annotation(
      Line(points = {{24, -52}, {69, -52}, {69, -42}, {110, -42}}, color = {0, 0, 127}, thickness = 0.5));
    connect(TankmultipleSOC, transAllocat.SOC) annotation(
      Line(points = {{6, -120}, {6, -65}}, color = {0, 0, 127}, thickness = 0.5));
    connect(AltFuelTankSOC, alt_Gen_Allocator.SOC) annotation(
      Line(points = {{66, -120}, {66, -86}, {67, -86}, {67, -79}}, color = {0, 0, 127}));
    connect(alt_Gen_Allocator.P_out, P_alt_dispatch) annotation(
      Line(points = {{80, -70}, {96, -70}, {96, -72}, {110, -72}}, color = {0, 0, 127}));
    connect(SOC_Tank, methanolBatteryDieselDebug.SOC_tank) annotation(
      Line(points = {{-10, 118}, {-10, 82}, {-40, 82}, {-40, 66}}, color = {0, 0, 127}));
    connect(methanolBatteryDieselDebug.P_fuelCellGen, ControllerFC_OutPut) annotation(
      Line(points = {{-4, 38}, {10, 38}, {10, 90}, {-36, 90}, {-36, 110}}, color = {0, 0, 127}));
    connect(methanolBatteryDieselDebug.P_dieselGen, transAllocat.P) annotation(
      Line(points = {{-2, 24}, {12, 24}, {12, -22}, {-20, -22}, {-20, -52}, {-4, -52}}, color = {0, 0, 127}));
    connect(methanolBatteryDieselDebug.P_altFuelGen, alt_Gen_Allocator.P) annotation(
      Line(points = {{-4, 14}, {44, 14}, {44, -70}, {60, -70}}, color = {0, 0, 127}));
    connect(P_load_1, methanolBatteryDieselDebug.P_load_1) annotation(
      Line(points = {{-120, 90}, {-82, 90}, {-82, 56}, {-62, 56}}, color = {0, 0, 127}));
    connect(P_renew_1, methanolBatteryDieselDebug.P_renew_1) annotation(
      Line(points = {{-120, 62}, {-94, 62}, {-94, 46}, {-62, 46}}, color = {0, 0, 127}));
    connect(SOC_1, methanolBatteryDieselDebug.SOC_1) annotation(
      Line(points = {{-120, 30}, {-86, 30}, {-86, 36}, {-62, 36}}, color = {0, 0, 127}));
    connect(altFuelUpperBound, methanolBatteryDieselDebug.altFuelOptimUpperBound) annotation(
      Line(points = {{-112, -12}, {-86, -12}, {-86, 20}, {-62, 20}}, color = {0, 0, 127}));
    connect(altFuelLowerBound, methanolBatteryDieselDebug.altFuelOptimLowerBound) annotation(
      Line(points = {{-112, -28}, {-80, -28}, {-80, 14}, {-62, 14}}, color = {0, 0, 127}));
    connect(altFuelMinBSFC, methanolBatteryDieselDebug.altFuelPwrAtMinBSFC) annotation(
      Line(points = {{-112, -46}, {-76, -46}, {-76, 24}, {-62, 24}}, color = {0, 0, 127}));
    connect(methanolBatteryDieselDebug.dieselPwrAtMinBSFC, dieselMinimalBSFC) annotation(
      Line(points = {{-58, 4}, {-58, -92}, {-108, -92}}, color = {0, 0, 127}));
    connect(dieselUpperBound, methanolBatteryDieselDebug.dieselOptimUpperBound) annotation(
      Line(points = {{-108, -62}, {-46, -62}, {-46, 4}}, color = {0, 0, 127}));
    connect(dieselLowerBound, methanolBatteryDieselDebug.dieselOptimLowerBound) annotation(
      Line(points = {{-108, -78}, {-36, -78}, {-36, 4}}, color = {0, 0, 127}));
    connect(gain1.y, P_charge_1) annotation(
      Line(points = {{80, 83}, {110, 83}, {110, 78}}, color = {0, 0, 127}));
    connect(methanolBatteryDieselDebug.P_battery_1, gain.u) annotation(
      Line(points = {{-4, 46}, {52, 46}, {52, 39}, {61, 39}}, color = {0, 0, 127}));
    connect(IceChargingPower.y, gain1.u) annotation(
      Line(points = {{40, 94}, {54, 94}, {54, 83}, {73, 83}}, color = {0, 0, 127}));
    annotation(
      uses(Modelica(version = "4.0.0")),
      Diagram,
      Icon(graphics = {Ellipse(origin = {50, -24}, fillPattern = FillPattern.Solid, extent = {{-26, 24}, {26, -24}}), Ellipse(origin = {0, 66}, fillColor = {141, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-26, 24}, {26, -24}}), Ellipse(origin = {-54, -22}, fillPattern = FillPattern.Solid, extent = {{-26, 24}, {26, -24}})}));
  end MasterControllerSingleBattery;

  /*Battery Dual Control*/

  model BatteryDualControl
    //Model Repository for Local BatteryDualControl Use

    partial model BatteryBase "Battery"
      parameter Modelica.Units.SI.Energy capacity(displayUnit = "kWh", min = capacity_min, max = capacity_max) = 20*3600*1e6 "Capacity [Ws]";
      parameter Boolean capacity_free_ = false "If true, then capacity is free in the optimization" annotation(
        Dialog(group = "Design", tab = "Optimization"));
      parameter Boolean use_SOC_constraint = true "If true, SOC is constrained in the optimization" annotation(
        Dialog(group = "Constraints", tab = "Optimization"));
      parameter Real SOC_min(unit = "1") = 0.1 "Minimum State Of Charge" annotation(
        Dialog(enable = use_SOC_constraint, group = "Constraints", tab = "Optimization"));
      parameter Real SOC_max(unit = "1") = 0.9 "Maximum State Of Charge" annotation(
        Dialog(enable = use_SOC_constraint, group = "Constraints", tab = "Optimization"));
      parameter Boolean set_SOC_final_start_ = false "If true, SOC at final time equals start value" annotation(
        Dialog(group = "Constraints", tab = "Optimization"));
      parameter Modelica.Units.SI.Power P_max = 2e5 "Maximum charging rate [W]" annotation(
        Dialog(group = "Control"));
      parameter Modelica.Units.SI.Power P_min = -P_max "Maximum discharging rate [W]" annotation(
        Dialog(group = "Control"));
      parameter Real SOC_start(unit = "1") = 0.5 annotation(
        Dialog(group = "Initialization"));
      Modelica.Units.SI.Energy charge(min = SOC_min*capacity, max = SOC_max*capacity) "Battery Charge";
      Real SOC(unit = "1") "State of Charge";
      Modelica.Units.SI.Power P_out "Total power into battery storage (internal losses between storage power and connector power)";
      Modelica.Electrical.Analog.Interfaces.PositivePin p annotation(
        Placement(transformation(extent = {{90, -10}, {110, 10}}), iconTransformation(extent = {{90, -10}, {110, 10}})));
      parameter Modelica.Units.SI.Energy capacity_min = 1e-3 "Minimum capacity in the optimization" annotation(
        Dialog(enable = capacity_free_, group = "Design", tab = "Optimization"));
      parameter Modelica.Units.SI.Energy capacity_max = 200*3600*1e6 "Maximum maximum capacity in the optimization" annotation(
        Dialog(enable = capacity_free_, group = "Design", tab = "Optimization"));
      Modelica.Units.SI.Power P_loss "Power loss from (dis)charging";
    equation
      SOC*capacity = charge;
      capacity*der(SOC) = P_out;
      assert(SOC >= 0, "Error: Battery is empty in " + getInstanceName() + " at time = " + String(time), level = AssertionLevel.error);
      assert((SOC <= 1), "Error: Battery reached maximum level in " + getInstanceName() + " at time = " + String(time), level = AssertionLevel.error);
    initial equation
      SOC = SOC_start;
    end BatteryBase;

    extends BatteryBase;
    parameter Real eff_charge(unit = "1") = 0.9 "Charge effciency";
    parameter Real eff_discharge(unit = "1") = eff_charge "Discharge efficiency";
    Modelica.Blocks.Interfaces.RealInput P_charge "Charging rate [p.u.]" annotation(
      Placement(transformation(origin = {-80, 40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-80, 38}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput P_discharge "Discharging rate [p.u.]" annotation(
      Placement(transformation(origin = {-80, -40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-80, -40}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Units.SI.Power P_charge_abs "Denormalized charging rate";
    Modelica.Units.SI.Power P_discharge_abs "Denormalized charging rate";
  equation
    P_charge_abs = P_charge*(P_max - 0) + 0;
    P_discharge_abs = P_discharge*(-P_min - 0) + 0;
    P_loss = (1 - eff_charge)*P_charge_abs + (1/eff_discharge - 1)*P_discharge_abs;
    P_out = P_charge_abs - P_discharge_abs - P_loss;
    p.i*p.v = P_charge_abs - P_discharge_abs;
    annotation(
      uses(Modelica(version = "4.0.0")),
      Icon(graphics = {Rectangle(origin = {4, -6}, fillPattern = FillPattern.VerticalCylinder, lineThickness = 1.25, extent = {{-54, 48}, {54, -48}}, radius = 4), Rectangle(origin = {-19, 51}, extent = {{-15, 7}, {15, -7}}), Rectangle(origin = {-19, 51}, lineThickness = 3, extent = {{-15, 7}, {15, -7}}, radius = 2), Rectangle(origin = {23, 51}, lineThickness = 3, extent = {{-15, 7}, {15, -7}}, radius = 3)}),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end BatteryDualControl;

  /*ElectricalLoad*/

  model ElectricLoad
    //Connects Pin_AC used by ElectricLoad

    connector Pin_AC "Pin of an electrical component"
      Modelica.Units.SI.Voltage v "Potential at the pin" annotation(
        unassignedMessage = "An electrical potential cannot be uniquely calculated.
    The reason could be that
    - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
      to define the zero potential of the electrical circuit, or
    - a connector of an electrical component is not connected.");
      flow Modelica.Units.SI.Current i "Current flowing into the pin" annotation(
        unassignedMessage = "An electrical current cannot be uniquely calculated.
    The reason could be that
    - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
      to define the zero potential of the electrical circuit, or
    - a connector of an electrical component is not connected.");
      annotation(
        defaultComponentName = "pin",
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Ellipse(extent = {{100, 100}, {-100, -100}}, lineColor = {0, 140, 72}, fillColor = {0, 140, 72}, fillPattern = FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
    end Pin_AC;

    parameter Boolean use_input = true "If true, load is an input";
    input Real load_in = 0 "Load by input in [W], only used if use_input is false" annotation(
      Dialog(enable = (not use_input)));
    Modelica.Blocks.Interfaces.RealOutput P_load "Connector of Real output signal" annotation(
      Placement(transformation(extent = {{100, 50}, {120, 70}})));
    Pin_AC n annotation(
      Placement(transformation(extent = {{90, -10}, {110, 10}}), iconTransformation(extent = {{90, -10}, {110, 10}})));
    Modelica.Blocks.Interfaces.RealInput P if use_input annotation(
      Placement(transformation(extent = {{-120, -16}, {-80, 24}})));
  protected
    Modelica.Blocks.Sources.RealExpression realExpression(y = load_in) if not use_input annotation(
      Placement(transformation(extent = {{-100, 74}, {-80, 94}})));
  equation
    n.i = P_load/n.v;
    connect(realExpression.y, P_load) annotation(
      Line(points = {{-79, 84}, {12, 84}, {12, 60}, {110, 60}}, color = {0, 0, 127}));
    connect(P, P_load) annotation(
      Line(points = {{-100, 4}, {12, 4}, {12, 60}, {110, 60}}, color = {0, 0, 127}));
    annotation(
      Placement(transformation(extent = {{-120, -12}, {-80, 30}})),
      Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, radius = 20), Rectangle(extent = {{-60, -24}, {56, -62}}, lineColor = {175, 175, 175}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, radius = 20), Rectangle(extent = {{-60, 14}, {56, -44}}, lineColor = {175, 175, 175}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-80, 28}, {76, 6}}, lineColor = {175, 175, 175}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, radius = 10), Rectangle(extent = {{-31, 8}, {31, -8}}, lineColor = {0, 128, 255}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, radius = 10, origin = {-28, 51}, rotation = 90, pattern = LinePattern.None), Rectangle(extent = {{-31, 8}, {31, -8}}, lineColor = {175, 175, 175}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, radius = 10, origin = {28, 51}, rotation = 90), Text(extent = {{-100, -110}, {100, -130}}, lineColor = {0, 0, 0}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, textString = "%name")}),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end ElectricLoad;

  /*ElectricalGrid*/

  model ElectricalGrid
    //Model Repository for Local ElectricalGrid use

    model ElectricGrid_base
      partial model TemplateSource_AC
        //Pin_AC used by TemplateSource_AC

        connector Pin_AC "Pin of an electrical component"
          Modelica.Units.SI.Voltage v "Potential at the pin" annotation(
            unassignedMessage = "An electrical potential cannot be uniquely calculated.
        The reason could be that
        - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
          to define the zero potential of the electrical circuit, or
        - a connector of an electrical component is not connected.");
          flow Modelica.Units.SI.Current i "Current flowing into the pin" annotation(
            unassignedMessage = "An electrical current cannot be uniquely calculated.
        The reason could be that
        - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
          to define the zero potential of the electrical circuit, or
        - a connector of an electrical component is not connected.");
          annotation(
            defaultComponentName = "pin",
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Ellipse(extent = {{100, 100}, {-100, -100}}, lineColor = {0, 140, 72}, fillColor = {0, 140, 72}, fillPattern = FillPattern.Solid)}),
            Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
        end Pin_AC;

        parameter Modelica.Units.SI.Voltage V_ref(displayUnit = "kV") = 20e3 "Grid reference AC voltage";
        parameter Boolean use_V_in = false "If true, voltage is an input" annotation(
          choices(checkBox = true),
          Dialog(tab = "Interfaces", group = "Inputs"));
        Modelica.Units.SI.Power P_out "Power output from grid";
        Pin_AC p annotation(
          Placement(transformation(extent = {{-110, -10}, {-90, 10}}), iconTransformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Blocks.Interfaces.RealInput V_in if use_V_in annotation(
          Placement(transformation(extent = {{126, 26}, {86, 66}})));
      protected
        Modelica.Blocks.Interfaces.RealOutput V_set_node annotation(
          Placement(transformation(extent = {{6, 64}, {-14, 84}})));
        Modelica.Blocks.Sources.RealExpression V_parameter(y = 20000) if not use_V_in annotation(
          Placement(transformation(extent = {{74, 58}, {42, 90}})));
      equation
        p.v = V_set_node;
        P_out = -p.v*p.i;
        connect(V_parameter.y, V_set_node) annotation(
          Line(points = {{40, 74}, {-4, 74}}, color = {0, 0, 127}));
        connect(V_in, V_set_node) annotation(
          Line(points = {{106, 46}, {24, 46}, {24, 74}, {-4, 74}}, color = {0, 0, 127}));
      end TemplateSource_AC;

      // Function max_approx used by ElectricGrid_base

      function max_approx "Max function approximation with continuous derivatives"
        input Real u1 "Argument 1";
        input Real u2 "Argument 2";
        input Real du = 0.1 "Smoothing interval, active when u1-u2 is within +/- 0.5*du";
        output Real y "max(u1,u2)";
      protected
        Real uu = -(u1 - u2)/du + 0.5 "Normalized position in smoothing interval";
      algorithm
        y := -du*noEvent(if uu < 0 then uu - 0.5 else (if uu > 1 then 0 else uu*(1 - uu*uu + 0.5*uu^3) - 0.5)) + u2;
      end max_approx;

      // Function min_approx used by ElectricGrid_base

      function min_approx "Min function approximation with continuous deriatives"
        input Real u1 "Argument 1";
        input Real u2 "Argument 2";
        input Real du = 0.1 "Smoothing interval, active when u1-u2 is within +/- 0.5*du";
        output Real y "min(u1,u2)";
      protected
        Real uu = (u1 - u2)/du + 0.5 "Normalized position in smoothing interval";
      algorithm
        y := u2 + du*noEvent(if uu < 0 then uu - 0.5 else (if uu > 1 then 0 else uu*(1 - uu*uu + 0.5*uu^3) - 0.5));
      end min_approx;

      //Repsostroy End
      type PerPower = Real(final quantity = "PerPower");
      type PerPower_PerkW = PerPower(final unit = "1/kW");
      //Properties of ElectricGrid_base
      extends TemplateSource_AC;
      parameter Real factor_sellprice = 1 "Quota between sell price and purchase price" annotation(
        Dialog(tab = "Economy"));
      parameter PerPower_PerkW gridPrice = 1 "Electric grid price in [1/kWh], only used if use_price_input is false" annotation(
        Dialog(tab = "Economy"));
      parameter Boolean use_price_input = true "Use input for electric grid price" annotation(
        choices(checkBox = true),
        Dialog(tab = "Interfaces", group = "Inputs"));
      parameter Modelica.Units.SI.Power power_nominal = 1e5 "Nominal power, for smoothing purposes" annotation(
        Dialog(tab = "Numerics"));
      Modelica.Blocks.Interfaces.RealOutput current_price "Current electrical price from grid" annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-46, 110}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-46, 110})));
      Real opex "Operational cost, per second";
      Modelica.Blocks.Interfaces.RealInput price if use_price_input annotation(
        Placement(transformation(extent = {{126, -20}, {86, 20}})));
    protected
      parameter Modelica.Units.SI.Power power_threshold = power_nominal/100;
      Modelica.Blocks.Sources.RealExpression price_parameter(y = gridPrice) if not use_price_input annotation(
        Placement(transformation(extent = {{2, 12}, {-34, 42}})));
    equation
      opex = (current_price*max_approx(P_out - power_threshold/2, 0, power_threshold))/3600/1000 + (min_approx(P_out + power_threshold/2, 0, power_threshold)*current_price*factor_sellprice)/3600/1000;
      connect(price_parameter.y, current_price) annotation(
        Line(points = {{-35.8, 27}, {-46, 27}, {-46, 110}}, color = {0, 0, 127}));
      connect(price, current_price) annotation(
        Line(points = {{106, 0}, {-46, 0}, {-46, 110}}, color = {0, 0, 127}));
    end ElectricGrid_base;

    model OptimizationConstraint "Block for time-invariant inequality constraints in the optimization"
      parameter Boolean exp_constraint_active = true "Use this parameter to turn on and off the constraint";
      parameter Real min_val = -Modelica.Constants.inf "Minimum value of exp" annotation(
        Dialog(group = "Constraining values"));
      parameter Real max_val = Modelica.Constants.inf "Maximum value of exp" annotation(
        Dialog(group = "Constraining values"));
      input Real exp(min = if exp_constraint_active then min_val else -Modelica.Constants.inf, max = if exp_constraint_active then max_val else Modelica.Constants.inf) "Expression to be constrained" annotation(
        Dialog(group = "Expression", enable = exp_constraint_active));
    equation
      if exp_constraint_active then
        assert(exp >= min_val, "Minimum constraint violated in " + getInstanceName() + " at t=" + String(time) + ": Adjust the constraint or control strategy to ensure the problem is feasible", level = AssertionLevel.warning);
        assert(exp <= max_val, "Maximum constraint violated in " + getInstanceName() + " at t=" + String(time) + ": Adjust the constraint or control strategy to ensure the problem is feasible", level = AssertionLevel.warning);
      end if;
      annotation(
        defaultComponentName = "constraint_",
        Icon(graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid), Text(extent = {{-80, 40}, {80, -40}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, textString = "Constraint"), Text(extent = {{-112, 32}, {112, -32}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, textString = "%name", origin = {-2, 58})}));
    end OptimizationConstraint;

    //Properties of ElectricalGrid
    extends ElectricGrid_base(V_parameter(y = 20000));
    OptimizationConstraint constraint_importExport(exp = P_out, min_val = -P_export_max, max_val = P_import_max) if use_power_constraints annotation(
      Placement(transformation(extent = {{26, -46}, {46, -26}})));
    parameter Modelica.Units.SI.Power P_peak = 1e9 "Peak power over time horizon (used in optimization)" annotation(
      Dialog(group = "Power peak", tab = "Optimization", enable = false));
    parameter Boolean P_peak_free_ = false "If true, then power peak is free in the optimization" annotation(
      Dialog(group = "Power peak", tab = "Optimization"));
    parameter Boolean use_power_constraints = false "If true, power import/export is constrained in the optimization" annotation(
      Dialog(group = "Import/export constraints", tab = "Optimization"));
    parameter Modelica.Units.SI.Power P_export_max = Modelica.Constants.inf "Maximal power export" annotation(
      Dialog(enable = use_power_constraints, group = "Import/export constraints", tab = "Optimization"));
    parameter Modelica.Units.SI.Power P_import_max = Modelica.Constants.inf "Maximal power import" annotation(
      Dialog(enable = use_power_constraints, group = "Import/export constraints", tab = "Optimization"));
  equation

  end ElectricalGrid;

  /*Converter_ACDC*/

  model Converter_ACDC
    // Converter component used by Converter ACDC

    partial model Converter
      type Time_yr = Real(final quantity = "Time", final unit = "yr");
      type PerPower = Real(final quantity = "PerPower");
      type PerPower_PerkW = PerPower(final unit = "1/kW");
      type PerPowerTime = Real(final quantity = "perPowerTime");
      type PerPowerTime_PerkWyr = PerPowerTime(final unit = "1/(kW.yr)");

      function sign_approx "Sign function with C2-continuous approximation "
        input Real u "Variable to take sign of";
        input Real eps = 1e-3 "Smoothing epsilon";
        output Real y "Approximated sign(u)";
      algorithm
        y := u/sqrt(u^2 + eps^2);
      end sign_approx;

      function heaviside_approx "Heaviside step function with C2-continuous approximation "
        input Real u "Variable to take Heaviside of";
        input Real eps = 1e-3 "Smoothing epsilon";
        output Real y "Approximated heaviside(u)";
      algorithm
        y := (sign_approx(u, eps) + 1)/2;
      end heaviside_approx;

      parameter Real efficiency = 0.99 "Effciency";
      parameter Modelica.Units.SI.Power P_max = 2e5 "Maximum source power [W]";
      parameter Boolean P_max_free_ = false "If true then P_max is a design parameter";
      Modelica.Units.SI.Power power_prim(start = 0, max = P_max, min = -P_max);
      Modelica.Units.SI.Power power_sec(start = 0, max = P_max, min = -P_max);
      Modelica.Units.SI.Power loss;
      parameter Time_yr lifetime = 10 "Expected lifetime [yr]";
      parameter PerPower_PerkW capex_p = 0 "CAPEX per maximum power [kW]";
      parameter PerPowerTime_PerkWyr fixed_opex_p = 0 "OPEX per kW per year [1/(kWyr)]";
      parameter Modelica.Units.SI.Power power_nominal = 1e6 "nominal power, for smoothing purpose ";
    equation
      loss = (1 - efficiency)*power_prim*heaviside_approx(power_prim, power_nominal/50) + (1 - efficiency)*power_sec*heaviside_approx(power_sec, power_nominal/50);
      power_prim + power_sec = loss;
    end Converter;

    //DefaultFlow Component used by Converter_ACDC

    model DefaultFlow
      type DefaultFlowPosition = enumeration(NONE, NORTH, NORTHEAST, EAST, SOUTHEAST, SOUTH, SOUTHWEST, WEST, NORTHWEST, EAST80, WEST80, EAST60, WEST60);
      parameter DefaultFlowPosition defaultFlow = if set_DC_voltage then DefaultFlowPosition.WEST80 else DefaultFlowPosition.EAST80 "Position of the connector that provides the flow that is being externally defined";
    end DefaultFlow;

    //Pin_AC Connecters used by Converter_ACDC

    connector Pin_AC "Pin of an electrical component"
      Modelica.Units.SI.Voltage v "Potential at the pin" annotation(
        unassignedMessage = "An electrical potential cannot be uniquely calculated.
    The reason could be that
    - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
      to define the zero potential of the electrical circuit, or
    - a connector of an electrical component is not connected.");
      flow Modelica.Units.SI.Current i "Current flowing into the pin" annotation(
        unassignedMessage = "An electrical current cannot be uniquely calculated.
    The reason could be that
    - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
      to define the zero potential of the electrical circuit, or
    - a connector of an electrical component is not connected.");
      annotation(
        defaultComponentName = "pin",
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Ellipse(extent = {{100, 100}, {-100, -100}}, lineColor = {0, 140, 72}, fillColor = {0, 140, 72}, fillPattern = FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
    end Pin_AC;

    //eleIn Connecters used by Converter_ACDC

    connector elecIn "Electrical reference values received through connector"
      connector RealConnector = Real "'Real' as connector" annotation(
        defaultComponentName = "u",
        Icon(graphics = {Rectangle(extent = {{100, -100}, {-100, 100}}, lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid)}, coordinateSystem(extent = {{-100.0, -100.0}, {100.0, 100.0}}, preserveAspectRatio = true, initialScale = 0.2)),
        Diagram(coordinateSystem(preserveAspectRatio = true, initialScale = 0.2, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Text(lineColor = {0, 0, 127}, extent = {{-10.0, 60.0}, {-10.0, 85.0}}, textString = "%name"), Rectangle(extent = {{100, -50}, {0, 50}}, lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid)}));
      // Maximum power point performances
      input RealConnector i_mp(final quantity = "ElectricCurrent", final unit = "A") "Maximum power point current";
      input RealConnector v_mp(final quantity = "ElectricPotential", final unit = "V") "Maximum power point voltage";
      input RealConnector P_mp(final quantity = "Power", final unit = "W") "Maximum power point power";
      // Actual power
      input RealConnector P(final quantity = "Power", final unit = "W") "Actual power";
      annotation(
        Icon(coordinateSystem(initialScale = 0.2), graphics = {Ellipse(extent = {{-10, 10}, {10, -10}}, lineColor = {0, 0, 0}, lineThickness = 0.5), Ellipse(extent = {{-50, 50}, {50, -50}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, lineThickness = 0.5), Ellipse(extent = {{-40, 40}, {40, -40}}, lineColor = {95, 95, 95}, fillColor = {255, 204, 51}, fillPattern = FillPattern.Solid, lineThickness = 0.5), Polygon(points = {{-20, 80}, {20, 80}, {0, 44}, {-20, 80}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, lineThickness = 0.5)}),
        Diagram(coordinateSystem(initialScale = 0.2)));
    end elecIn;

    type DefaultFlowPosition = enumeration(NONE, NORTH, NORTHEAST, EAST, SOUTHEAST, SOUTH, SOUTHWEST, WEST, NORTHWEST, EAST80, WEST80, EAST60, WEST60);
    //properties belongs to Converter
    extends Converter;
    //properties belongs to DefaultFlow
    final DefaultFlowPosition defaultFlow = if set_DC_voltage then DefaultFlowPosition.WEST80 else DefaultFlowPosition.EAST80;
    //properties belongs to Converter_ACDC
    parameter Boolean set_DC_voltage = true "If true, voltage will be set at DC pin";
    parameter Modelica.Units.SI.Voltage V_ref_DC = 48 "Reference DC source voltage on DC pin" annotation(
      Dialog(enable = not use_V_in and not set_DC_voltage));
    parameter Modelica.Units.SI.Voltage V_ref_AC = 48 "Reference AC source voltage on AC pin" annotation(
      Dialog(enable = not use_V_in and not set_DC_voltage));
    parameter Boolean use_V_in = false "if true, DC voltage is an input" annotation(
      choices(checkBox = true),
      Dialog(tab = "Interfaces", group = "Inputs"));
    parameter Boolean use_pvInfo = false "true to set the voltage through pVInfo, false for connecting Real signal" annotation(
      choices(checkBox = true),
      Dialog(tab = "Interfaces", group = "Inputs"));
    Pin_AC pin_AC annotation(
      Placement(transformation(extent = {{70, -10}, {90, 10}}), iconTransformation(extent = {{70, -10}, {90, 10}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_DC annotation(
      Placement(transformation(extent = {{-90, -10}, {-70, 10}}), iconTransformation(extent = {{-90, -10}, {-70, 10}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y = if set_DC_voltage then V_ref_DC else V_ref_AC) if not use_V_in annotation(
      Placement(transformation(origin = {0, -6}, extent = {{-92, 82}, {-72, 102}})));
    Modelica.Blocks.Interfaces.RealInput V_in if (not use_pvInfo) and (use_V_in) "Voltage setpint, DC if set_DC_voltage ==true, else AC voltage" annotation(
      Placement(transformation(extent = {{-20.0, -20.0}, {20.0, 20.0}}, rotation = -90.0, origin = {2, 112}), iconTransformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {4, 80})));
    elecIn pVInfo if use_pvInfo and use_V_in "PhotoVoltaic information through port" annotation(
      Placement(transformation(origin = {-4.8, 12.3}, extent = {{-27.2, 35.7}, {6.8, 69.7}}), iconTransformation(extent = {{-32, 42}, {8, 82}})));
  protected
    Modelica.Blocks.Interfaces.RealInput V_node "DC Voltage used" annotation(
      Placement(transformation(origin = {24, 38}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {26, 40}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  equation
    if set_DC_voltage then
      pin_DC.v = V_node;
    else
      pin_AC.v = V_node;
    end if;
    power_prim = pin_AC.v*pin_AC.i;
    power_sec = pin_DC.v*pin_DC.i;
    connect(V_node, V_in) annotation(
      Line(points = {{24, 38}, {24, 78}, {2, 78}, {2, 112}}, color = {0, 0, 127}));
    connect(V_node, pVInfo.v_mp) annotation(
      Line(points = {{24, 38}, {24, 78}, {-22, 78}, {-22, 65}, {-15, 65}}, color = {0, 0, 127}));
    connect(realExpression.y, V_node) annotation(
      Line(points = {{-71, 86}, {-38, 86}, {-38, 38}, {24, 38}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(extent = {{-80, 60}, {80, -60}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, radius = 5), Line(points = {{-74, -56}, {76, 56}}, color = {0, 0, 0}), Text(extent = {{4, -10}, {84, -50}}, lineColor = {0, 140, 72}, textString = "~"), Text(extent = {{-86, 56}, {-6, 16}}, lineColor = {0, 0, 255}, textString = "="), Text(extent = {{-80, -66}, {80, -80}}, lineColor = {0, 0, 0}, textString = "%name")}),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Converter_ACDC;

  /*Transformer*/

  model Transformer
    partial model Converter
      type Time_yr = Real(final quantity = "Time", final unit = "yr");
      type PerPower = Real(final quantity = "PerPower");
      type PerPower_PerkW = PerPower(final unit = "1/kW");
      type PerPowerTime = Real(final quantity = "perPowerTime");
      type PerPowerTime_PerkWyr = PerPowerTime(final unit = "1/(kW.yr)");

      function sign_approx "Sign function with C2-continuous approximation "
        input Real u "Variable to take sign of";
        input Real eps = 1e-3 "Smoothing epsilon";
        output Real y "Approximated sign(u)";
      algorithm
        y := u/sqrt(u^2 + eps^2);
      end sign_approx;

      function heaviside_approx "Heaviside step function with C2-continuous approximation "
        input Real u "Variable to take Heaviside of";
        input Real eps = 1e-3 "Smoothing epsilon";
        output Real y "Approximated heaviside(u)";
      algorithm
        y := (sign_approx(u, eps) + 1)/2;
      end heaviside_approx;

      parameter Real efficiency = 0.99 "Effciency";
      parameter Modelica.Units.SI.Power P_max = 2e5 "Maximum source power [W]";
      parameter Boolean P_max_free_ = false "If true then P_max is a design parameter";
      Modelica.Units.SI.Power power_prim(start = 0, max = P_max, min = -P_max);
      Modelica.Units.SI.Power power_sec(start = 0, max = P_max, min = -P_max);
      Modelica.Units.SI.Power loss;
      parameter Time_yr lifetime = 10 "Expected lifetime [yr]";
      parameter PerPower_PerkW capex_p = 0 "CAPEX per maximum power [kW]";
      parameter PerPowerTime_PerkWyr fixed_opex_p = 0 "OPEX per kW per year [1/(kWyr)]";
      parameter Modelica.Units.SI.Power power_nominal = 1e6 "nominal power, for smoothing purpose ";
    equation
      loss = (1 - efficiency)*power_prim*heaviside_approx(power_prim, power_nominal/50) + (1 - efficiency)*power_sec*heaviside_approx(power_sec, power_nominal/50);
      power_prim + power_sec = loss;
    end Converter;

    model DefaultFlow
      type DefaultFlowPosition = enumeration(NONE, NORTH, NORTHEAST, EAST, SOUTHEAST, SOUTH, SOUTHWEST, WEST, NORTHWEST, EAST80, WEST80, EAST60, WEST60);
      parameter DefaultFlowPosition defaultFlow = if set_DC_voltage then DefaultFlowPosition.WEST80 else DefaultFlowPosition.EAST80 "Position of the connector that provides the flow that is being externally defined";
    end DefaultFlow;

    connector Pin_AC "Pin of an electrical component"
      Modelica.Units.SI.Voltage v "Potential at the pin" annotation(
        unassignedMessage = "An electrical potential cannot be uniquely calculated.
    The reason could be that
    - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
      to define the zero potential of the electrical circuit, or
    - a connector of an electrical component is not connected.");
      flow Modelica.Units.SI.Current i "Current flowing into the pin" annotation(
        unassignedMessage = "An electrical current cannot be uniquely calculated.
    The reason could be that
    - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
      to define the zero potential of the electrical circuit, or
    - a connector of an electrical component is not connected.");
      annotation(
        defaultComponentName = "pin",
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Ellipse(extent = {{100, 100}, {-100, -100}}, lineColor = {0, 140, 72}, fillColor = {0, 140, 72}, fillPattern = FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
    end Pin_AC;

    // Propertied belongs to Transformer
    extends Converter;
    Pin_AC pin_prim annotation(
      Placement(transformation(extent = {{-10.0, 90.0}, {10.0, 110.0}}, rotation = 0.0, origin = {0.0, 0.0})));
    Pin_AC pin_sec annotation(
      Placement(transformation(extent = {{-10.0, -110.0}, {10.0, -90.0}}, rotation = 0.0, origin = {0.0, 0.0})));
    parameter Modelica.Units.SI.Voltage V_ref = 48 "Reference AC source voltage on secondary pin.";
  equation
    pin_sec.v = V_ref;
    power_prim = pin_prim.v*pin_prim.i;
    power_sec = pin_sec.v*pin_sec.i;
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(extent = {{-100, 10}, {100, -10}}, lineColor = {0, 0, 0}, textString = "%name", origin = {-82, 0}, rotation = 90), Rectangle(extent = {{-60, -80}, {60, -94}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-54, 40}, {54, -68}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Polygon(points = {{-6, 22}, {12, 22}, {4, -2}, {26, -2}, {-2, -44}, {2, -14}, {-20, -14}, {-6, 22}}, lineColor = {255, 255, 255}, pattern = LinePattern.None, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-6, 84}, {6, 46}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-10, 56}, {10, 52}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-10, 68}, {10, 62}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-10, 78}, {10, 74}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-50, 78}, {-30, 74}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-50, 68}, {-30, 62}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-50, 56}, {-30, 52}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-46, 84}, {-34, 46}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{34, 84}, {46, 46}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{30, 56}, {50, 52}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{30, 68}, {50, 62}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{30, 78}, {50, 74}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid)}),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Transformer;

  /*Fuel*/

  package Fuel
    // External Model Repository for Local Use
    /*SimpleFuel*/

    package Templates
      package SimpleFuel
        //      extends Modelon.Icons.Record;
        constant Modelica.Units.SI.SpecificEnergy LHV = 1 "Lower Heating value";
        constant Modelica.Units.SI.Density rho = 1 "Density";
        constant Modelica.Units.SI.Density rho_liq = 1 "Density";
        constant Modelica.Units.SI.MassFraction carbonContent = 1 "kg carbon per kg fuel";
        constant .Modelica.Units.SI.MolarMass MolarMass = 1 "Molar mass of fuel";
        constant Modelica.Units.SI.MolarEnergy Delta_h_evap = 1 "Molar evaporation enthalpy";
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {95, 95, 95}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Polygon(points = {{100, 20}, {40, 22}, {34, 34}, {-12, 24}, {-10, 16}, {-42, -2}, {-82, -52}, {-72, -58}, {-30, -10}, {-6, -2}, {16, -30}, {82, -48}, {82, 0}, {100, -2}, {100, 20}}, pattern = LinePattern.None, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, lineColor = {0, 0, 0}), Polygon(points = {{72, 0}, {34, 6}, {16, -4}, {22, -12}, {36, -6}, {72, -22}, {72, 0}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Polygon(points = {{28, -20}, {36, -14}, {70, -30}, {28, -20}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = false)),
          Documentation(revisions = "<html>
                Copyright &copy; 2004-2024, MODELON AB <br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br />This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br />or by other means have their origin from any Modelon Library.
                </html>", info = "<html>
                Base record for fuel properties.
                </html>"));
      end SimpleFuel;
    end Templates;

    /*CompressedHydrogen*/

    package compressedhydrogen
      extends AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.MethanolFuel(LHV = 120e6, rho = 26, rho_liq = 26, MolarMass = 1.00899e-3, Delta_h_evap = 2.01588*1e-3);
    end compressedhydrogen;

    /*Diesel*/

    package Diesel "Diesel fuel properties"
      extends Fuel.Templates.SimpleFuel(carbonContent = 0.86, LHV = 44e6, rho = 820, rho_liq = 820, MolarMass = 0.233, Delta_h_evap = 400e3*MolarMass);
    end Diesel;

    /*Methanol Fuel*/

    package MethanolFuel "Methanol Fuel for diesel engine"
      extends Fuel.Diesel(carbonContent = 0, LHV = 20.1e6, rho_liq = 791.4, rho = 791.4, MolarMass = 32.04e-3, Delta_h_evap = 715000);
    end MethanolFuel;

    /*NatualGas*/

    package NaturalGas "Methan / Natural  Gas fuel properties"
      extends Fuel.Templates.SimpleFuel(carbonContent = 0.75, LHV = 47e6, rho = 0.78, rho_liq = 422.6, MolarMass = 16.0425e-3, Delta_h_evap = 508.82e3*MolarMass);
    end NaturalGas;

    /*Hydrogen*/

    package Hydrogen "Hydrogen fuel properties"
      extends Fuel.Templates.SimpleFuel(carbonContent = 0, LHV = 120e6, rho = 0.08988, rho_liq = 70.85, MolarMass = 2.01588*1e-3, Delta_h_evap = 0.9e3);
    end Hydrogen;

    /*CarbonDioxide*/

    package CarbonDioxide "Carbon Dioxide fuel properties"
      extends Fuel.Templates.SimpleFuel(carbonContent = 12/44, LHV = 0, rho = 1.977, rho_liq = 1101, MolarMass = 44.009*1e-3, Delta_h_evap = 179.5e3*MolarMass);
    end CarbonDioxide;

    // Repositroy end Fuel
    annotation(
      Diagram,
      Icon(graphics = {Bitmap(origin = {2, 2}, rotation = 180, extent = {{-100, 140}, {100, -140}}, imageSource = "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")}));
  end Fuel;

  /*Generator*/

  model Generator
    //External Model Repositroy for Local Generator USe
    /*Generator_Base*/

    model Generator_Base
      // External Model and packge Repository for Local Generator_Base use
      /*DefaultFlow*/

      model DefaultFlow
        type DefaultFlowPosition = enumeration(NONE, NORTH, NORTHEAST, EAST, SOUTHEAST, SOUTH, SOUTHWEST, WEST, NORTHWEST, EAST80, WEST80, EAST60, WEST60);
        parameter DefaultFlowPosition defaultFlow = DefaultFlowPosition.NONE "Position of the connector that provides the flow that is being externally defined";
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Rectangle(origin = {0, 100}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = defaultFlow == DefaultFlowPosition.NORTH), Rectangle(origin = {100, 100}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.NORTHEAST)), Rectangle(origin = {100, 0}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.EAST)), Rectangle(origin = {100, -100}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.SOUTHEAST)), Rectangle(origin = {0, -100}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.SOUTH)), Rectangle(origin = {-100, -100}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.SOUTHWEST)), Rectangle(origin = {-100, 0}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.WEST)), Rectangle(origin = {-100, 100}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.NORTHWEST)), Rectangle(origin = {80, 0}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.EAST80)), Rectangle(origin = {-80, 0}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.WEST80)), Rectangle(origin = {60, 0}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.EAST60)), Rectangle(origin = {-60, 0}, extent = {{13, 13}, {-13, -13}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, visible = (defaultFlow == DefaultFlowPosition.WEST60))}));
      end DefaultFlow;

      /*Fuel*/
      // Current reside in Generator model due to  scoping reaasons
      /*VloumnFlowRate*/
      type VolumeFlowRate_lperh = Real(quantity = "VolumeFlowRate", final unit = "l/h");
      /*MassFlowRate*/
      type MassFlowRate_kgperh = Real(quantity = "MassFlowRate", final unit = "kg/h");
      parameter .Modelica.Units.SI.SpecificEnergy FLHV = 44e6 "Fuel LHV";
      parameter .Modelica.Units.SI.Density Frho = 820 "Fuel Density";
      parameter Real Frho_liq = 820 "Fuel Liquid Density";
      parameter Real FcarbonContent = 0.86 "Fuel Carbon Content";
      parameter .Modelica.Units.SI.MolarMass FMolarMass = 0.233 "Fuel MolarMaxx";
      parameter .Modelica.Units.SI.MolarEnergy FDelta_h_evap = 400e3*FMolarMass "Fuel Molar Energy";
      /*Converstion from_lperh*/

      function from_lperh "From l/h to m3/s"
        extends Modelica.Units.Icons.Conversion;
        input VolumeFlowRate_lperh lperh;
        output Modelica.Units.SI.VolumeFlowRate m3pers;
      algorithm
        m3pers := lperh/3600/1000;
      end from_lperh;

      /*Converstion from_kgperh*/

      function from_kgperh "From kg/h to kg/s"
        extends Modelica.Units.Icons.Conversion;
        input MassFlowRate_kgperh kgperh;
        output Modelica.Units.SI.MassFlowRate kgpers;
      algorithm
        kgpers := kgperh/3600;
      end from_kgperh;

      /*PerVolume*/
      type PerVolume = Real(final quantity = "PerVolume");
      /*PerPowerTime*/
      type PerPowerTime = Real(final quantity = "PerPowerTime");
      /*PerPower*/
      type PerPower = Real(final quantity = "PerPower");
      /*Time_yr*/
      type Time_yr = Real(final quantity = "Time", final unit = "yr");
      /*PerPower_PerkW*/
      type PerPower_PerkW = PerPower(final unit = "1/kW");
      /*PerPowerTime_PerkWyr*/
      type PerPowerTime_PerkWyr = PerPowerTime(final unit = "1/(kW.yr)");
      /*PerVolume_l*/
      type PerVolume_l = PerVolume(final unit = "1/l");
      /*Modelon.Units.Conversions.Null*/

      function Null "Null conversion"
        extends Modelica.Units.Icons.Conversion;
        input Real inPut "input value";
        output Real outPut "output value";
      algorithm
        outPut := inPut;
      end Null;

      /*Modelon conversion_from_kgph*/

      block Conversion "Conversion between SI and non-SI units"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock;
        replaceable function conversion = Null constrainedby Modelica.Units.Icons.Conversion "Conversion function" annotation(
           choicesAllMatching);
      equation
        y = conversion(u);
      end Conversion;

      /*Heat Port a*/

      connector HeatPort_a "Heat flow connector with outlined icon"
        extends Modelica.Thermal.HeatTransfer.Interfaces.HeatPort;
        annotation(
          defaultComponentName = "port_a",
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-80, 80}, {80, -80}}, lineColor = {191, 0, 0}, fillColor = {191, 0, 0}, fillPattern = FillPattern.Solid), Text(extent = {{-100, 120}, {100, 80}}, lineColor = {191, 0, 0}, textString = "%name")}),
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {191, 0, 0}, fillColor = {191, 0, 0}, fillPattern = FillPattern.Solid), Text(extent = {{-98, 196}, {102, 102}}, lineColor = {191, 0, 0}, textString = "%name")}));
      end HeatPort_a;

      /*Fluid Port*/

      connector FluidPort "Fluid port for low fidelity models in microgrid components"
        replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Hydrogen constrainedby AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.SimpleFuel "Transported medium";
        Modelica.Units.SI.Pressure p "Pressure";
        flow Modelica.Units.SI.MassFlowRate m_flow "Mass flow rate";
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Ellipse(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end FluidPort;

      /*pin_AC*/

      connector Pin_AC "Pin of an electrical component"
        Modelica.Units.SI.Voltage v "Potential at the pin" annotation(
          unassignedMessage = "An electrical potential cannot be uniquely calculated.
      The reason could be that
      - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
        to define the zero potential of the electrical circuit, or
      - a connector of an electrical component is not connected.");
        flow Modelica.Units.SI.Current i "Current flowing into the pin" annotation(
          unassignedMessage = "An electrical current cannot be uniquely calculated.
      The reason could be that
      - a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
        to define the zero potential of the electrical circuit, or
      - a connector of an electrical component is not connected.");
        annotation(
          defaultComponentName = "pin",
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Ellipse(extent = {{100, 100}, {-100, -100}}, lineColor = {0, 140, 72}, fillColor = {0, 140, 72}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
      end Pin_AC;

      /*cubicStep*/

      function cubicStep "Cubic step function"
        input Real tau "Abcissa";
        output Real y "Value";
      algorithm
        y := if tau < 0 then 0 else (if tau > 1 then 1 else (3 - 2*tau)*tau^2);
      end cubicStep;

      /*signApprox*/

      function sign_approx "Sign function with C2-continuous approximation "
        input Real u "Variable to take sign of";
        input Real eps = 1e-3 "Smoothing epsilon";
        output Real y "Approximated sign(u)";
      algorithm
        y := u/sqrt(u^2 + eps^2);
      end sign_approx;

      // Repository End
      extends DefaultFlow(final defaultFlow = if use_P_in then DefaultFlow.DefaultFlowPosition.NONE else DefaultFlow.DefaultFlowPosition.WEST);
      parameter Boolean P_rat_free = false "If true, rated power P-rat is free(requires use_const_eff=true)" annotation(
        Dialog(enable = use_const_eff, group = "Design", tab = "Optimization"));
      parameter Boolean use_P_in = true "true of power setpoint is an input";
      parameter Modelica.Units.SI.Power P_rat = 800*1000 "Generator capacity";
      parameter Boolean use_external_co2_sink = false "True if CO2 connector is enabled" annotation(
        Dialog(tab = "Interfaces"));
      parameter Boolean use_external_fuel_source = false "True if CO2 connector is enabled" annotation(
        Dialog(tab = "Interfaces"));
      parameter Boolean disable = false "Disable component";
      final parameter Real availability = if disable then 0 else 1;
      replaceable package Fuel = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Diesel constrainedby AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.SimpleFuel "Fuel type" annotation(
         choicesAllMatching,
         Placement(transformation(extent = {{-82.0, 58.0}, {-62.0, 78.0}}, rotation = 0.0, origin = {0.0, 0.0})),
         Dialog(tab = "Fuel Consumption"));
      Modelica.Blocks.Tables.CombiTable1Dv power_to_fuelconsump(table = power_vs_fuelconsum, tableOnFile = false) if use_fuel_eff_table "kW vs l/h" annotation(
        Placement(transformation(extent = {{-10.0, -10.0}, {10.0, 10.0}}, rotation = -90.0, origin = {-16.0, -16.0})));
      parameter Boolean use_const_eff = false "if true then using constant efficiency" annotation(
        Dialog(enable = not use_fuel_eff_table, tab = "Fuel Consumption"));
      parameter Real eta_el_const(min = 0, max = 1) = 0.33 "Constant electric efficiency" annotation(
        Dialog(enable = use_const_eff, tab = "Fuel Consumption"));
      parameter Boolean use_fuel_eff_table = true "if true, fuel efficiency as a table, defined with F1 otherwise" annotation(
        Dialog(tab = "Fuel Consumption", enable = not use_const_eff));
      parameter Boolean fuelConsumptionIsVolumetric = true "Fuel amount for fuel consumption in liter, otherwise kg" annotation(
        Evaluate = true,
        Dialog(enable = not use_const_eff, tab = "Fuel Consumption"));
      parameter Modelica.Units.SI.VolumeFlowRate V_flow_fidle(displayUnit = "l/h") "Fuel consumption at idle operation" annotation(
        Dialog(enable = fuelConsumptionIsVolumetric and ((not use_fuel_eff_table) and not use_const_eff), tab = "Fuel Consumption"));
      parameter Modelica.Units.SI.VolumeFlowRate V_flow_frat(displayUnit = "l/h") "Fuel consumption at peak operation" annotation(
        Dialog(enable = fuelConsumptionIsVolumetric and (not use_fuel_eff_table) and not use_const_eff, tab = "Fuel Consumption"));
      parameter Modelica.Units.SI.MassFlowRate m_flow_fidle(displayUnit = "kg/h") = from_kgperh(0.08451/1000*P_rat) "Fuel consumption at idle operation" annotation(
        Dialog(enable = not fuelConsumptionIsVolumetric and (not use_fuel_eff_table) and not use_const_eff, tab = "Fuel Consumption"));
      parameter Modelica.Units.SI.MassFlowRate m_flow_frat(displayUnit = "kg/h") = from_kgperh((0.08451 + 0.25)/1000*P_rat) "Fuel consumption at peak operation" annotation(
        Dialog(enable = not fuelConsumptionIsVolumetric and (not use_fuel_eff_table) and not use_const_eff, tab = "Fuel Consumption"));
      //26,37.38; 50, 64.54; 81,103.98; 100,134.15
      parameter Real power_vs_fuelconsum[:, :] = [0, 20.35; 10, 38.75; 20, 56.48; 30, 73.78; 40, 90.78; 50, 107.53; 60, 124.09; 70, 140.48; 80, 156.72; 90, 172.83; 100, 182.77] "Percentage of nominal power vs fuel consumption table [l/h] or [kg/h]" annotation(
        Dialog(enable = use_fuel_eff_table and not use_const_eff, tab = "Fuel Consumption"));
      // [0,20.35; 10,38.75; 20,56.48; 30,73.78; 40,90.78; 50,107.53; 60,124.09; 70,140.48; 80,156.72; 90,172.83; 100,182.77]
      // Economy Related Parameters
      parameter Time_yr lifetime = 16 "Expected lifetime [yr]" annotation(
        Dialog(tab = "Economy"));
      parameter PerPower_PerkW capex_p = 0 "CAPEX per rated power [1/kW]" annotation(
        Dialog(tab = "Economy"));
      parameter PerPowerTime_PerkWyr fixed_opex_p = 0 "OPEX per kW per year [1/(kWyr)]" annotation(
        Dialog(tab = "Economy"));
      parameter PerVolume_l fuel_cost = 0 "Cost per liter of fuel [1/l]" annotation(
        Dialog(tab = "Economy", enable = not use_external_fuel_source));
      parameter Real co2_emission_cost = 0 "Cost per ton of carbondioxide [1/t]" annotation(
        Dialog(tab = "Economy", enable = not use_external_co2_sink));
      parameter Boolean use_CO2_constraint = false "If true, CO2 emission constrained in the optimization" annotation(
        Dialog(group = "Emission limits", tab = "Optimization"));
      parameter Modelica.Units.SI.Mass CO2_max = Modelica.Constants.inf "Maximum CO2 emission, in kg" annotation(
        Dialog(enable = use_CO2_constraint, group = "Emission limits", tab = "Optimization"));
      parameter Boolean limit_operation_time = false "If true, running hours constrained in the optimization" annotation(
        Dialog(group = "Other limits", tab = "Optimization"));
      parameter Modelica.Units.SI.Time max_operation_hours(displayUnit = "h") = Modelica.Constants.inf "Maximum hours to run during optimization" annotation(
        Dialog(enable = limit_operation_time, group = "Other limits", tab = "Optimization"));
      Conversion conversion_from_kgph(redeclare replaceable function conversion = from_kgperh) if use_fuel_eff_table and not fuelConsumptionIsVolumetric annotation(
        Placement(transformation(extent = {{8.661555581893829, -37.33844441810617}, {19.33844441810617, -26.66155558189383}}, origin = {0.0, 0.0}, rotation = 0.0)));
      Conversion conversion_from_lph(redeclare replaceable function conversion = from_lperh) if use_fuel_eff_table and fuelConsumptionIsVolumetric annotation(
        Placement(transformation(extent = {{8.661555581893829, -21.33844441810617}, {19.33844441810617, -10.661555581893829}}, origin = {0.0, 0.0}, rotation = 0.0)));
      parameter Boolean use_combined_heat_and_power = false "Add heatport" annotation(
        Dialog(tab = "Interfaces"));
      parameter Real eta_tot(min = eta_el_const, max = 1) = 1 "Total System Efficiency" annotation(
        Dialog(tab = "Combined Heat and Power"));
      FluidPort fluidPort(redeclare package Medium = Fuel, m_flow = m_flow_fuel) if use_external_fuel_source "FluidPort for fuel" annotation(
        Placement(transformation(extent = {{90, -10}, {110, 10}}, origin = {0, 0}, rotation = 0)));
      //Modelica.Units.SI.TimeAging variable_opex = fuel_cost*Modelica.Units.Conversions.to_litre(V_flow_fuel) + m_flow_CO2/1000*co2_emission_cost "Cost of fuel consumption and CO2 emmisions per second" annotation(Dialog(tab = "Economy"));
      Real P_gen(unit = "1", min = 0, max = 1) "Normalized power output";
      Modelica.Units.SI.Power P_out "Electrical power output";
      Modelica.Units.SI.Power Q_flow "Thermal power output";
      Modelica.Units.SI.Power P_loss "Power loss";
      Modelica.Units.SI.MassFlowRate m_flow_fuel(min = 0) "Fuel mass flow";
      Modelica.Units.SI.VolumeFlowRate V_flow_fuel(min = 0) "Fuel volume flow";
      Modelica.Units.SI.MassFlowRate m_flow_CO2(min = 0) "CO2 emission";
      Modelica.Units.SI.Time operation_time(displayUnit = "h") "Operation time";
      Modelica.Units.SI.Mass mass_CO2(min = 0) "Accumulated CO2 emissions";
      Modelica.Blocks.Interfaces.RealInput P_sp(min = 0, nominal = 1) if use_P_in "Power setpoint in p.u." annotation(
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 270, origin = {0, 100})));
      Pin_AC pin_AC annotation(
        Placement(transformation(extent = {{-110, -10}, {-90, 10}}), iconTransformation(extent = {{-110, -10}, {-90, 10}})));
      Modelica.Blocks.Math.Gain PercentNominalP(k = 100) if use_fuel_eff_table annotation(
        Placement(transformation(extent = {{-6.0, -6.0}, {6.0, 6.0}}, rotation = -90.0, origin = {-16.0, 12.0})));
      FluidPort co2_outlet(redeclare package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.CarbonDioxide, m_flow = -m_flow_CO2) if use_external_co2_sink annotation(
        Placement(transformation(extent = {{-10.0, -110.0}, {10.0, -90.0}}, rotation = 0.0, origin = {0.0, 0.0})));
    protected
      constant Modelica.Units.SI.MolarMass MM_C = Modelica.Media.IdealGases.Common.SingleGasesData.C.MM "Cabon Molar Mass";
      constant Modelica.Units.SI.MolarMass MM_O2 = Modelica.Media.IdealGases.Common.SingleGasesData.O2.MM "Cabon Molar Mass";
      constant Modelica.Units.SI.MolarMass MM_CO2 = Modelica.Media.IdealGases.Common.SingleGasesData.CO2.MM "Cabon Molar Mass";
    protected
      Modelica.Blocks.Interfaces.RealOutput flow_fnode annotation(
        Placement(transformation(extent = {{46.0, -42.0}, {66.0, -22.0}}, rotation = 0.0, origin = {0.0, 0.0})));
      Modelica.Blocks.Sources.RealExpression Vfuel_parameter(y = V_flow_frat) if not use_fuel_eff_table and fuelConsumptionIsVolumetric and not use_const_eff annotation(
        Placement(transformation(extent = {{-72, -54}, {-40, -26}})));
      Modelica.Blocks.Sources.RealExpression mfuel_parameter1(y = m_flow_frat) if not use_const_eff and not use_fuel_eff_table and not fuelConsumptionIsVolumetric annotation(
        Placement(transformation(extent = {{-72, -78}, {-40, -50}})));
      Modelica.Blocks.Interfaces.RealOutput P_node annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {16, 2})));
    equation
      if use_combined_heat_and_power then
        Q_flow = (m_flow_fuel*FLHV*eta_tot - P_out);
        P_loss = (m_flow_fuel*FLHV - P_out - Q_flow);
      else
        Q_flow = (m_flow_fuel*FLHV - P_out);
        P_loss = Q_flow;
      end if;
      if limit_operation_time then
        der(operation_time) = cubicStep(P_gen);
      else
        operation_time = 0;
      end if;
      P_gen = P_node*availability;
      P_out = P_gen*P_rat;
      m_flow_fuel = V_flow_fuel*Frho;
//efficiency=P_out/m_flow_fuel/fuel.LHV;
      if use_const_eff then
        eta_el_const*m_flow_fuel*FLHV = P_out;
        flow_fnode = 0;
// for mathematical stability , if constant efficiency is used this variable is not used in computation but it should be assigned with something for numerical balance
      else
        if fuelConsumptionIsVolumetric then
          V_flow_fuel = conversion_from_lph.y;
        else
          m_flow_fuel = conversion_from_kgph.y;
        end if;
      end if;
      m_flow_CO2 = FcarbonContent*m_flow_fuel*(1 + MM_O2/MM_C);
      if use_CO2_constraint then
        der(mass_CO2) = m_flow_CO2;
      else
        mass_CO2 = 0;
      end if;
      -pin_AC.i*pin_AC.v = P_out;
      connect(PercentNominalP.y, power_to_fuelconsump.u[1]) annotation(
        Line(points = {{-16, 5.4}, {-16, -4}, {-15.999999999999996, -4}}, color = {0, 0, 127}));
      connect(Vfuel_parameter.y, flow_fnode) annotation(
        Line(points = {{-38.4, -40}, {-38.4, -46}, {26, -46}, {26, -32}, {56, -32}}, color = {0, 0, 127}));
      connect(PercentNominalP.u, P_sp) annotation(
        Line(points = {{-15.999999999999998, 19.2}, {-15.999999999999998, 74}, {0, 74}, {0, 100}}, color = {0, 0, 127}));
      connect(mfuel_parameter1.y, flow_fnode) annotation(
        Line(points = {{-38.4, -64}, {26, -64}, {26, -32}, {56, -32}}, color = {0, 0, 127}));
      connect(P_sp, P_node) annotation(
        Line(points = {{0, 100}, {0, 51}, {16, 51}, {16, 2}}, color = {0, 0, 127}));
      connect(power_to_fuelconsump.y[1], conversion_from_kgph.u) annotation(
        Line(points = {{-16.000000000000004, -27}, {-16.000000000000004, -32}, {7.593866698272595, -32}}, color = {0, 0, 127}));
      connect(conversion_from_kgph.y, flow_fnode) annotation(
        Line(points = {{19.872288859916786, -32}, {56, -32}}, color = {0, 0, 127}));
      connect(conversion_from_lph.y, flow_fnode) annotation(
        Line(points = {{19.872288859916786, -16}, {26, -16}, {26, -32}, {56, -32}}, color = {0, 0, 127}));
      connect(conversion_from_lph.u, power_to_fuelconsump.y[1]) annotation(
        Line(points = {{7.593866698272595, -16}, {1.5938666982725946, -16}, {1.5938666982725946, -32}, {-15.999999999999998, -32}, {-15.999999999999998, -27}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, radius = 20), Rectangle(extent = {{-78, 40}, {90, -40}}, lineColor = {51, 51, 51}, fillColor = {51, 51, 51}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-6, 78}, {82, 8}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{0, 70}, {76, 12}}, lineColor = {51, 51, 51}, fillColor = {51, 51, 51}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-92, 20}, {-84, -20}}, lineColor = {51, 51, 51}, fillColor = {51, 51, 51}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-76, -46}, {-48, -64}}, lineColor = {51, 51, 51}, fillColor = {51, 51, 51}, fillPattern = FillPattern.Solid), Rectangle(extent = {{60, -44}, {88, -62}}, lineColor = {51, 51, 51}, fillColor = {51, 51, 51}, fillPattern = FillPattern.Solid), Polygon(points = {{38, 60}, {30, 40}, {40, 44}, {36, 30}, {34, 34}, {36, 24}, {42, 30}, {38, 30}, {44, 48}, {34, 44}, {42, 60}, {38, 60}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-72, 32}, {-34, 28}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-72, 22}, {-34, 18}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-72, 12}, {-34, 8}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-72, -30}, {-34, -34}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-72, -20}, {-34, -24}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-72, -10}, {-34, -14}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-300, -100}, {300, -130}}, lineColor = {0, 0, 0}, fillColor = {0, 115, 200}, fillPattern = FillPattern.Solid, textString = "%name")}),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        __Dymola_experimentSetupOutput,
        __Dymola_experimentFlags(Advanced(GenerateVariableDependencies = false, OutputModelicaCode = false), Evaluate = false, OutputCPUtime = false, OutputFlatModelica = false));
    end Generator_Base;

    extends Generator_Base(final use_P_in = true, P_rat(min = P_rat_min, max = P_rat_max), use_external_co2_sink = false, use_const_eff = true, disable = false);
    parameter Boolean P_rat_free_ = false "If true rated power P_rat is free" annotation(
      Dialog(group = "Design", tab = "Optimization"));
    parameter Modelica.Units.SI.Power P_rat_min = 0 "Minimal rated power" annotation(
      Dialog(enable = P_rat_free_, group = "Design", tab = "Optimization"));
    parameter Modelica.Units.SI.Power P_rat_max = .Modelica.Constants.inf "Maximal rated power" annotation(
      Dialog(enable = P_rat_free_, group = "Design", tab = "Optimization"));
    FluidPort outlet_fuel(m_flow = m_flow_fuel) annotation(
      Placement(transformation(extent = {{-54.0, -110.0}, {-34.0, -90.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
  equation

  end Generator;

  /*IdealTankMultiPort*/

  model IdealTankMultiPort
    //External Models repository for Local Use

    model TriplePortTank
      //External Models Respoitory for Local TriplePortTank use
      /*FluidPort */

      connector FluidPort "Fluid port for low fidelity models in microgrid components"
        replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Hydrogen constrainedby AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.SimpleFuel "Transported medium";
        Modelica.Units.SI.Pressure p "Pressure";
        flow Modelica.Units.SI.MassFlowRate m_flow "Mass flow rate";
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Ellipse(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end FluidPort;

      /*Fuel*/
      //Fuel model will be referenced From AZEAT_FortunaCrane_Short_ControllerGroup
      replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Hydrogen constrainedby AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Templates.SimpleFuel "Transported medium" annotation(
         choicesAllMatching);
      parameter Boolean use_p_in = true "User input connector" annotation(
        choices(checkBox = true));
      input Modelica.Units.SI.Pressure p = 101325 "Boundry Pressure" annotation(
        Dialog(enable = not use_p_in));
      FluidPort fluidPort(redeclare package Medium = Medium) annotation(
        Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
      Modelica.Blocks.Interfaces.RealInput p_in if use_p_in annotation(
        Placement(transformation(extent = {{140, -20}, {100, 20}})));
      FluidPort fluidPort2(redeclare package Medium = Medium) annotation(
        Placement(transformation(extent = {{-110.0, 20.0}, {-90.0, 40.0}}, rotation = 0.0, origin = {0.0, 0.0})));
      FluidPort fluidPort3(redeclare package Medium = Medium) annotation(
        Placement(transformation(origin = {2, 0}, extent = {{-112, -36}, {-92, -16}}), iconTransformation(extent = {{-112, -36}, {-92, -16}})));
    protected
      Modelica.Blocks.Interfaces.RealInput p_internal;
    equation
      fluidPort.p = p_internal;
      fluidPort2.p = p_internal;
      fluidPort3.p = p_internal;
      if not use_p_in then
        p_internal = p;
      end if;
      connect(p_in, p_internal);
      annotation(
        Icon(graphics = {Ellipse(extent = {{-100, 100}, {100, -100}}, fillPattern = FillPattern.Solid, fillColor = {127, 202, 238}, lineColor = {51, 127, 216}), Text(textString = "p", origin = {0, 0}, extent = {{-80, 80}, {80, -80}}, lineColor = {74, 84, 226})}));
    end TriplePortTank;

    //Repository End
    extends TriplePortTank(use_p_in = false);
    parameter Modelica.Units.SI.Mass m_content_max(min = 0) = 100 "'Tank' content at the beginning of simulation" annotation(
      Dialog(group = "Dimension"));
    parameter Modelica.Units.SI.Mass m_content_start(min = 0, max = m_content_max) = 100 "'Tank' content at the beginning of simulation" annotation(
      Dialog(group = "Initialization"));
    Modelica.Units.SI.Mass m_content(start = m_content_start);
    Real SOC = m_content/m_content_max;
  equation
    der(m_content) = fluidPort.m_flow + fluidPort2.m_flow + fluidPort3.m_flow;
    annotation(
      Icon(graphics = {Ellipse(origin = {2, -78}, extent = {{-101, 24}, {101, -24}}, fillPattern = FillPattern.Solid, fillColor = {74, 144, 226}), Rectangle(origin = {2, 5}, extent = {{-100, 81}, {100, -81}}, fillPattern = FillPattern.Solid, fillColor = {74, 144, 226}, lineColor = {74, 144, 226}), Ellipse(origin = {2, 84}, extent = {{-101, 24}, {101, -24}}, fillPattern = FillPattern.Solid, fillColor = {74, 144, 226})}),
      uses(Modelica(version = "4.0.0"), ThermalPower(version = "1.25")));
  end IdealTankMultiPort;

  model HardCodeDutyCycle "Hard code the duty cycle into the componenet, so that no external csv references are nedded"
    import Modelica.Blocks.Tables.CombiTable1Ds;
    //Define the duty cycle
    parameter Real dutyCycleRaw[449, 2] = [0, 0.411519447; 60, 0.411519447; 120, 0.476772092; 180, 0.466705548; 240, 0.370768416; 300, 0.556563893; 360, 250; 420, 250; 480, 250; 540, 250; 600, 250; 660, 250; 720, 250; 780, 250; 840, 148.5612282; 900, 206.1434123; 960, 179.7993104; 1020, 88.34563361; 1080, 95.3150035; 1140, 92.43223317; 1200, 100.6309441; 1260, 99.87128108; 1320, 104.4690219; 1380, 103.3732366; 1440, 102.8199147; 1500, 105.6392742; 1560, 110.672483; 1620, 115.5967626; 1680, 111.4797109; 1740, 110.2378546; 1800, 107.0979566; 1860, 109.0807124; 1920, 108.1114858; 1980, 117.1553802; 2040, 117.2917675; 2100, 123.45711; 2160, 118.1825761; 2220, 119.7365337; 2280, 115.9099912; 2340, 120.6732228; 2400, 125.4892383; 2460, 133.5747478; 2520, 107.7509764; 2580, 105.3048575; 2640, 100.8707356; 2700, 100.590498; 2760, 91.09763288; 2820, 92.69413079; 2880, 94.87010193; 2940, 91.29699915; 3000, 58.91577323; 3060, 53.00210275; 3120, 62.75000479; 3180, 61.41852665; 3240, 59.20024957; 3300, 53.34480967; 3360, 57.48728932; 3420, 53.96534777; 3480, 54.48112934; 3540, 120.8924548; 3600, 125.2345059; 3660, 124.089546; 3720, 124.81336; 3780, 109.4194533; 3840, 114.2713998; 3900, 153.9831802; 3960, 161.6440358; 4020, 160.796975; 4080, 160.7150358; 4140, 155.1546241; 4200, 142.1135409; 4260, 152.7808232; 4320, 147.2894811; 4380, 150.5837555; 4440, 148.864816; 4500, 70.51595328; 4560, 72.51932993; 4620, 198.6493448; 4680, 190.1518318; 4740, 177.6865573; 4800, 250; 4860, 250; 4920, 250; 4980, 250; 5040, 115.3930135; 5100, 2.055400004; 5160, 1.716188916; 5220, 1.539200011; 5280, 1.633361096; 5340, 1.792577765; 5400, 1.689922222; 5460, 1.662908321; 5520, 1.581744449; 5580, 2.02564167; 5640, 2.141226295; 5700, 1.779597206; 5760, 1.924175017; 5820, 213.8644555; 5880, 250; 5940, 250; 6000, 250; 6060, 250; 6120, 250; 6180, 250; 6240, 250; 6300, 250; 6360, 250; 6420, 153.5361451; 6480, 108.0423872; 6540, 136.6084548; 6600, 142.7625915; 6660, 145.7976034; 6720, 143.7254677; 6780, 144.2172233; 6840, 147.7498071; 6900, 145.785734; 6960, 187.1252242; 7020, 250; 7080, 63.88758681; 7140, 123.8334418; 7200, 2.334194997; 7260, 2.562905557; 7320, 2.299205234; 7380, 3.326341675; 7440, 1.943579493; 7500, 2.15430443; 7560, 2.338188887; 7620, 2.20579447; 7680, 106.4368131; 7740, 44.61900873; 7800, 233.4351637; 7860, 235.1672732; 7920, 232.8986655; 7980, 231.6611891; 8040, 234.1281644; 8100, 232.3446125; 8160, 236.6685516; 8220, 237.4567937; 8280, 234.820983; 8340, 235.6139064; 8400, 239.2765668; 8460, 236.1924311; 8520, 250; 8580, 57.33389321; 8640, 28.81232558; 8700, 2.282713887; 8760, 2.32144168; 8820, 2.460522714; 8880, 2.539933357; 8940, 2.28273052; 9000, 57.03157239; 9060, 250; 9120, 250; 9180, 250; 9240, 250; 9300, 250; 9360, 250; 9420, 250; 9480, 250; 9540, 250; 9600, 250; 9660, 250; 9720, 250; 9780, 250; 9840, 250; 9900, 250; 9960, 109.4097501; 10020, 248.4788742; 10080, 210.441491; 10140, 210.9393742; 10200, 225.4252519; 10260, 250; 10320, 250; 10380, 250; 10440, 144.0721629; 10500, 200.7428232; 10560, 2.400652789; 10620, 2.638216688; 10680, 2.628697217; 10740, 2.345541653; 10800, 2.504402327; 10860, 2.777388891; 10920, 245.6594403; 10980, 187.6340752; 11040, 216.5618146; 11100, 219.7992925; 11160, 222.9196068; 11220, 220.8786926; 11280, 223.2635227; 11340, 218.9323149; 11400, 224.6313777; 11460, 222.8613887; 11520, 223.0134044; 11580, 224.8510806; 11640, 228.139392; 11700, 223.5070714; 11760, 250; 11820, 250; 11880, 154.2116173; 11940, 2.916613883; 12000, 2.725408893; 12060, 2.647148743; 12120, 2.863334198; 12180, 2.679691686; 12240, 3.398661163; 12300, 2.46172501; 12360, 2.669802344; 12420, 3.02649998; 12480, 2.653038899; 12540, 2.553224968; 12600, 2.68128058; 12660, 235.4504395; 12720, 250; 12780, 250; 12840, 250; 12900, 250; 12960, 250; 13020, 250; 13080, 250; 13140, 250; 13200, 250; 13260, 250; 13320, 250; 13380, 250; 13440, 250; 13500, 250; 13560, 250; 13620, 250; 13680, 199.8640008; 13740, 250; 13800, 250; 13860, 250; 13920, 250; 13980, 250; 14040, 250; 14100, 250; 14160, 250; 14220, 250; 14280, 250; 14340, 250; 14400, 250; 14460, 250; 14520, 250; 14580, 250; 14640, 250; 14700, 250; 14760, 250; 14820, 250; 14880, 250; 14940, 250; 15000, 250; 15060, 250; 15120, 250; 15180, 250; 15240, 231.522011; 15300, 2.801255577; 15360, 2.747000008; 15420, 2.948071775; 15480, 2.993452794; 15540, 2.816613926; 15600, 2.793358319; 15660, 3.067605542; 15720, 250; 15780, 250; 15840, 250; 15900, 250; 15960, 250; 16020, 250; 16080, 25.83916217; 16140, 250; 16200, 250; 16260, 250; 16320, 250; 16380, 250; 16440, 250; 16500, 250; 16560, 250; 16620, 250; 16680, 250; 16740, 250; 16800, 14.54737525; 16860, 45.02893678; 16920, 13.32913042; 16980, 11.24577223; 17040, 13.05656242; 17100, 11.75769465; 17160, 10.81227226; 17220, 10.40906101; 17280, 14.54645551; 17340, 13.37256393; 17400, 250; 17460, 250; 17520, 250; 17580, 230.7019195; 17640, 136.4175905; 17700, 250; 17760, 2.805159251; 17820, 177.8759952; 17880, 150.7971543; 17940, 138.6760955; 18000, 137.267785; 18060, 129.9033294; 18120, 128.1740054; 18180, 127.3520836; 18240, 121.2342731; 18300, 124.3122864; 18360, 114.8764408; 18420, 115.4593428; 18480, 118.4396102; 18540, 118.2521047; 18600, 250; 18660, 10.26434453; 18720, 230.4512146; 18780, 233.8279991; 18840, 250; 18900, 250; 18960, 250; 19020, 250; 19080, 250; 19140, 250; 19200, 250; 19260, 250; 19320, 250; 19380, 250; 19440, 250; 19500, 250; 19560, 250; 19620, 250; 19680, 250; 19740, 250; 19800, 250; 19860, 250; 19920, 250; 19980, 250; 20040, 250; 20100, 250; 20160, 250; 20220, 181.1442156; 20280, 116.3925181; 20340, 67.01614519; 20400, 146.1969401; 20460, 138.4525869; 20520, 138.8228298; 20580, 135.2087918; 20640, 141.2348919; 20700, 250; 20760, 250; 20820, 250; 20880, 250; 20940, 250; 21000, 53.09762343; 21060, 154.5833531; 21120, 96.96797551; 21180, 104.1210155; 21240, 206.9129024; 21300, 197.9193398; 21360, 192.9400849; 21420, 199.7791693; 21480, 200.1267376; 21540, 203.5746053; 21600, 200.726315; 21660, 204.8693191; 21720, 173.9018351; 21780, 178.540496; 21840, 196.0317612; 21900, 187.4426171; 21960, 180.0321415; 22020, 181.3408394; 22080, 189.7380091; 22140, 187.6296628; 22200, 177.5623758; 22260, 184.4655344; 22320, 185.7722012; 22380, 189.7481598; 22440, 190.2705137; 22500, 190.1372254; 22560, 188.1918814; 22620, 194.1985851; 22680, 190.2719888; 22740, 195.6748072; 22800, 195.661382; 22860, 196.0450257; 22920, 200.4427787; 22980, 195.5350338; 23040, 199.9211549; 23100, 205.0897111; 23160, 197.5451546; 23220, 194.6551484; 23280, 200.6799202; 23340, 202.3410561; 23400, 197.4569821; 23460, 193.3476044; 23520, 205.4653019; 23580, 212.7246628; 23640, 208.5308228; 23700, 205.5865309; 23760, 204.1463119; 23820, 207.0856607; 23880, 207.5338673; 23940, 202.3711145; 24000, 197.0176849; 24060, 205.7948938; 24120, 205.8928596; 24180, 210.0234752; 24240, 207.8325573; 24300, 134.8424546; 24360, 13.54121105; 24420, 18.40187488; 24480, 29.31576928; 24540, 31.6580003; 24600, 31.08371661; 24660, 38.16272238; 24720, 43.99765507; 24780, 40.66147285; 24840, 39.89059973; 24900, 41.60480833; 24960, 46.15445545; 25020, 241.6540642; 25080, 250; 25140, 250; 25200, 250; 25260, 250; 25320, 250; 25380, 250; 25440, 250; 25500, 250; 25560, 250; 25620, 250; 25680, 250; 25740, 250; 25800, 250; 25860, 250; 25920, 250; 25980, 250; 26040, 250; 26100, 250; 26160, 250; 26220, 250; 26280, 250; 26340, 250; 26400, 250; 26460, 250; 26520, 250; 26580, 250; 26640, 250; 26700, 250; 26760, 250; 26820, 250; 26880, 99.62749646];
    Modelica.Blocks.Tables.CombiTable1Ds dutyCycleTable(table = dutyCycleRaw, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints) annotation(
      Placement(transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Real y;
    Modelica.Blocks.Interfaces.RealOutput out annotation(
      Placement(transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-110, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  equation
    y = dutyCycleTable.y[1];
    dutyCycleTable.u = time;
    connect(dutyCycleTable.y[1], out) annotation(
      Line(points = {{-13, 0}, {-110, 0}}, color = {0, 0, 127}));
    annotation(
      uses(Modelica(version = "4.0.0")),
      Icon(graphics = {Text(origin = {-1, 6}, extent = {{-89, 60}, {89, -60}}, textString = "Duty Cycle Data"), Rectangle(origin = {-1, 3}, extent = {{-95, -23}, {95, 23}})}));
  end HardCodeDutyCycle;

  /*HydrogenTank*/

  model Tank "Tank model for gas storage"
    // External Models repository fir Local Tank use
    /*Tank Templates*/

    partial model Tank_base
      //External Function Repository for Local Tank_base use;

      connector FluidPort "Fluid port for low fidelity models in microgrid components"
        replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Hydrogen constrainedby AZEAT_FortunaCrane_Short_ControllerGroupFuel.Templates.SimpleFuel "Transported medium";
        Modelica.Units.SI.Pressure p "Pressure";
        flow Modelica.Units.SI.MassFlowRate m_flow "Mass flow rate";
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Ellipse(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end FluidPort;

      replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.compressedhydrogen "Transported Medium" annotation(
        choicesAllMatching);
      parameter Real SOC_start = 0.5;
      parameter Modelica.Units.SI.Volume V(min = V_min, max = V_max) = 1 "Tank volumn" annotation(
        Dialog(group = "Parameters"));
      parameter Boolean set_SOC_final_start_ = false "If true, SOC at final time equals start value" annotation(
        Dialog(group = "Constraints", tab = "Optimization"));
      Real SOC(start = SOC_start, fixed = true, min = 0, max = 1) "Tank SOC p_min->0, p_max->1";
      Modelica.Units.SI.AmountOfSubstance n "Amount of substance in tank";
      Modelica.Units.SI.Power P_loss "Power loss from leakage, based on H2 LHV";
      Modelica.Units.SI.Power P_in = m_flow_in*Medium.LHV "Energyflow into tank based on LHV";
      Modelica.Units.SI.Power P_out = m_flow_out*Medium.LHV "Energyflow from tank based on LHV";
      parameter Boolean V_free_ = false "If true, then volume is free in optimization" annotation(
        Dialog(group = "Design", tab = "Optimization"));
      parameter Modelica.Units.SI.Volume V_min = 1e-3 "Minimum volume in the optimization" annotation(
        Dialog(enable = V_free_, group = "Design", tab = "Optimization"));
      parameter Modelica.Units.SI.Volume V_max = 1e5 "Maximum volumein the optimization" annotation(
        Dialog(enable = V_free_, group = "Design", tab = "Optimization"));
      Modelica.Units.SI.Mass mass = n*Medium.MolarMass "Mass in tank";
      Modelica.Units.SI.MassFlowRate m_flow_in = inlet.m_flow "Mass flow in";
      Modelica.Units.SI.MassFlowRate m_flow_out = outlet.m_flow "Mass flow out";
      Modelica.Units.SI.MassFlowRate m_flow_loss "Mass flow lost to environment";
      Modelica.Units.SI.MolarFlowRate n_flow_in = m_flow_in/Medium.MolarMass "Molar flow in";
      Modelica.Units.SI.MolarFlowRate n_flow_out = m_flow_out/Medium.MolarMass "Molar flow out";
      parameter Modelica.Units.SI.Energy E_store "Energy storagy capacity";
      FluidPort inlet(redeclare package Medium = Medium) annotation(
        Placement(transformation(extent = {{-70, -10}, {-50, 10}}), iconTransformation(extent = {{-70, -10}, {-50, 10}})));
      FluidPort outlet(redeclare package Medium = Medium) annotation(
        Placement(transformation(extent = {{50, -10}, {70, 10}}), iconTransformation(extent = {{50, -10}, {70, 10}})));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-60, -100}, {60, 100}}), graphics = {Rectangle(extent = {{-24, 66}, {24, 56}}, lineColor = {0, 0, 0}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, radius = 30), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Rectangle(extent = {{-60, 60}, {60, -80}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, radius = 40), Rectangle(extent = {{-24, 90}, {24, 80}}, lineColor = {0, 0, 0}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, radius = 30), Rectangle(extent = {{-14, 80}, {14, 66}}, lineColor = {0, 0, 0}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid), Text(extent = {{-60, 34}, {60, -46}}, lineColor = {0, 128, 255}, fillColor = {170, 213, 255}, fillPattern = FillPattern.Solid, textString = "H2")}));
    end Tank_base;

    /*Constraint*/

    model Constraint "Block for time-invariant inequality constraints in the optimization"
      parameter Boolean exp_constraint_active = false "Use this parameter to turn on and off the constraint";
      parameter Real min_val = -Modelica.Constants.inf "Minimum value of exp" annotation(
        Dialog(group = "Constraining values"));
      parameter Real max_val = Modelica.Constants.inf "Maximum value of exp" annotation(
        Dialog(group = "Constraining values"));
      input Real exp(min = if exp_constraint_active then min_val else -Modelica.Constants.inf, max = if exp_constraint_active then max_val else Modelica.Constants.inf) "Expression to be constrained" annotation(
        Dialog(group = "Expression", enable = exp_constraint_active));
    equation
      if exp_constraint_active then
        assert(exp >= min_val, "Minimum constraint violated in " + getInstanceName() + " at t=" + String(time) + ": Adjust the constraint or control strategy to ensure the problem is feasible", level = AssertionLevel.warning);
        assert(exp <= max_val, "Maximum constraint violated in " + getInstanceName() + " at t=" + String(time) + ": Adjust the constraint or control strategy to ensure the problem is feasible", level = AssertionLevel.warning);
      end if;
      annotation(
        defaultComponentName = "constraint_",
        Icon(graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid), Text(extent = {{-80, 40}, {80, -40}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, textString = "Constraint"), Text(extent = {{-112, 32}, {112, -32}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, textString = "%name", origin = {-2, 58})}));
    end Constraint;

    //EndRepositroy
    extends Tank_base(E_store = energyDensity_max*V, redeclare replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.compressedhydrogen constrainedby AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Templates.SimpleFuel);
    //Local Parameters
    parameter Modelica.Units.SI.Temperature T = 273.15 + 80 "Tank temperature" annotation(
      Dialog(group = "Parameters"));
    parameter Modelica.Units.SI.MolarFlowRate ndot_leak = 1e-4 "Molar leak flow" annotation(
      Dialog(group = "Parameters"));
    //Local Limits
    parameter Boolean use_pressure_constraint = true "If true, SOC is constrained in the optimization" annotation(
      Dialog(group = "Limits"));
    parameter Modelica.Units.SI.Pressure p_max = 750e5 "Max tank pressure" annotation(
      Dialog(group = "Limits"));
    parameter Modelica.Units.SI.Pressure p_min = 10e5 "Min tank pressure" annotation(
      Dialog(group = "Limits"));
    // Conversion
    //  final parameter Modelica.Units.SI.SpecificEnergy LHV(displayUnit="kWh/kg") = Medium.LHV "Lower heating value";
    final parameter Modelica.Units.SI.Density rho_max = Medium.MolarMass*p_max/T/Modelica.Constants.R "Tank kg per m3";
    final parameter Modelica.Units.SI.EnergyDensity energyDensity_max = Medium.LHV*rho_max "Max. energy density";
    //variables
    Modelica.Units.SI.Pressure p "Tank pressure";
    Constraint constraint_SOC(exp = SOC, min_val = 0, max_val = 1) if use_pressure_constraint annotation(
      Placement(transformation(extent = {{10.0, 30.0}, {30.0, 50.0}}, rotation = 0.0, origin = {0.0, 0.0})));
  equation
    der(SOC) = (.Modelica.Constants.R*T/(V*(p_max - p_min)))*(inlet.m_flow/Medium.MolarMass + outlet.m_flow/Medium.MolarMass - ndot_leak);
    p = SOC*(p_max - p_min) + p_min;
    n = p*V/(.Modelica.Constants.R*T);
    m_flow_loss = ndot_leak*Medium.MolarMass;
// Losses
    P_loss = m_flow_loss*Medium.LHV;
// Connectors
    inlet.p = p;
    outlet.p = p;
    annotation(
      Diagram(coordinateSystem(extent = {{-60, -100}, {60, 100}})));
  end Tank;

  /*TransCellSpliter*/

  block TransCellsplitter
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(transformation(extent = {{-139.44179364012908, 38.55820635987092}, {-104.55820635987092, 73.44179364012908}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(transformation(extent = {{151.87045005112003, 63.870450051120045}, {188.12954994887997, 100.12954994887995}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Interfaces.RealInput SOC1 annotation(
      Placement(transformation(extent = {{-17.441793640129077, -17.441793640129077}, {17.441793640129077, 17.441793640129077}}, origin = {-80.0, 140.0}, rotation = -90.0)));
    Modelica.Blocks.Interfaces.RealInput SOC2 annotation(
      Placement(transformation(extent = {{-17.441793640129077, -17.441793640129077}, {17.441793640129077, 17.441793640129077}}, origin = {-6.0, 140.0}, rotation = -90.0)));
    Modelica.Blocks.Logical.Switch switch3 annotation(
      Placement(transformation(extent = {{107.16666666666666, 85.83333333333334}, {127.16666666666666, 105.83333333333334}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression SOC_1(y = SOC1) annotation(
      Placement(transformation(extent = {{41.166666666666686, 53.83333333333333}, {61.166666666666686, 73.83333333333333}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.Greater greater3 annotation(
      Placement(transformation(extent = {{72.5, 54.5}, {92.5, 74.5}})));
    Modelica.Blocks.Sources.RealExpression Min_SOC_Allowed(y = 0.1) annotation(
      Placement(transformation(extent = {{30.4456, 10}, {49.5544, 30}}, origin = {-8, 2})));
    Modelica.Blocks.Sources.RealExpression zval3(y = 0) annotation(
      Placement(transformation(extent = {{63.612246157867986, 77.83333333333334}, {82.72108717546533, 97.83333333333334}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Math.Division division2 annotation(
      Placement(transformation(extent = {{4.0, 38.0}, {24.0, 58.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression FCnumbers(y = 2) annotation(
      Placement(transformation(extent = {{-22.0, 12.0}, {-2.0, 32.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression zval5(y = 0) annotation(
      Placement(transformation(extent = {{65.61224615786799, 5.833333333333343}, {84.72108717546533, 25.833333333333343}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression SOC_2(y = SOC2) annotation(
      Placement(transformation(extent = {{43.166666666666686, -18.16666666666667}, {63.166666666666686, 1.8333333333333286}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.Greater greater4 annotation(
      Placement(transformation(extent = {{74.5, -17.499999999999986}, {94.5, 2.500000000000014}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.Switch switch4 annotation(
      Placement(transformation(extent = {{109.16666666666666, 13.833333333333343}, {129.16666666666666, 33.83333333333334}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Interfaces.RealOutput y2 annotation(
      Placement(transformation(extent = {{151.87045005112003, 25.870450051120045}, {188.12954994887997, 62.129549948879955}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.Switch switch annotation(
      Placement(transformation(extent = {{-52.66666666666667, 23.666666666666657}, {-32.66666666666667, 43.66666666666666}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.Greater greater annotation(
      Placement(transformation(extent = {{-87.33333333333333, -7.666666666666664}, {-67.33333333333333, 12.333333333333336}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression zval2(y = 250000) annotation(
      Placement(transformation(extent = {{-117.55442050879867, -32.0}, {-98.44557949120133, -12.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression MaxRatedPower(y = 0) annotation(
      Placement(transformation(extent = {{-92, 32}, {-72, 52}}, origin = {6, 6})));
  equation
    connect(SOC_1.y, greater3.u1) annotation(
      Line(points = {{62.1667, 63.8333}, {62.1667, 64.5}, {70.5, 64.5}}, color = {0, 0, 127}));
    connect(Min_SOC_Allowed.y, greater3.u2) annotation(
      Line(points = {{43, 22}, {43, 56.5}, {70.5, 56.5}}, color = {0, 0, 127}));
    connect(greater3.y, switch3.u2) annotation(
      Line(points = {{93.5, 64.5}, {98.6667, 64.5}, {98.6667, 95.8333}, {105.167, 95.8333}}, color = {255, 0, 255}));
    connect(zval3.y, switch3.u3) annotation(
      Line(points = {{83.6765292263452, 87.83333333333333}, {105.16666666666666, 87.83333333333333}}, color = {0, 0, 127}));
    connect(FCnumbers.y, division2.u2) annotation(
      Line(points = {{-1, 22}, {-1, 42}, {2, 42}}, color = {0, 0, 127}));
    connect(division2.y, switch3.u1) annotation(
      Line(points = {{25, 48}, {25, 103.83333333333334}, {105.16666666666666, 103.83333333333334}}, color = {0, 0, 127}));
    connect(greater4.y, switch4.u2) annotation(
      Line(points = {{95.49999999999999, -7.5}, {100.66666666666666, -7.5}, {100.66666666666666, 23.83333333333333}, {107.16666666666666, 23.83333333333333}}, color = {255, 0, 255}));
    connect(zval5.y, switch4.u3) annotation(
      Line(points = {{85.6765292263452, 15.833333333333329}, {107.16666666666666, 15.833333333333329}}, color = {0, 0, 127}));
    connect(division2.y, switch4.u1) annotation(
      Line(points = {{25, 48}, {25, 31.833333333333343}, {107.16666666666666, 31.833333333333343}}, color = {0, 0, 127}));
    connect(switch3.y, y) annotation(
      Line(points = {{128.16666666666666, 95.83333333333334}, {136, 95.83333333333334}, {136, 82}, {170, 82}}, color = {0, 0, 127}));
    connect(switch4.y, y2) annotation(
      Line(points = {{130.16666666666666, 23.833333333333343}, {134, 23.833333333333343}, {134, 44}, {170, 44}}, color = {0, 0, 127}));
    connect(SOC_2.y, greater4.u1) annotation(
      Line(points = {{64.16666666666666, -8.166666666666671}, {64.16666666666666, -7.5}, {72.49999999999999, -7.5}}, color = {0, 0, 127}));
    connect(Min_SOC_Allowed.y, greater4.u2) annotation(
      Line(points = {{43, 22}, {58, 22}, {58, -15.5}, {72.5, -15.5}}, color = {0, 0, 127}));
    connect(greater.y, switch.u2) annotation(
      Line(points = {{-66.33333333333333, 2.3333333333333144}, {-61.16666666666666, 2.3333333333333144}, {-61.16666666666666, 33.66666666666664}, {-54.66666666666666, 33.66666666666664}}, color = {255, 0, 255}));
    connect(zval2.y, greater.u2) annotation(
      Line(points = {{-97.49013744032146, -22}, {-97.49013744032146, -5.666666666666664}, {-89.33333333333333, -5.666666666666664}}, color = {0, 0, 127}));
    connect(u, greater.u1) annotation(
      Line(points = {{-122, 56}, {-105.66666666666666, 56}, {-105.66666666666666, 2.3333333333333357}, {-89.33333333333333, 2.3333333333333357}}, color = {0, 0, 127}));
    connect(u, switch.u3) annotation(
      Line(points = {{-122, 56}, {-122, 19.666666666666657}, {-54.66666666666667, 19.666666666666657}, {-54.66666666666667, 25.666666666666657}}, color = {0, 0, 127}));
    connect(switch.y, division2.u1) annotation(
      Line(points = {{-31.66666666666667, 33.66666666666666}, {-14.833333333333336, 33.66666666666666}, {-14.833333333333336, 54}, {2, 54}}, color = {0, 0, 127}));
    connect(MaxRatedPower.y, switch.u1) annotation(
      Line(points = {{-65, 48}, {-65, 47.6667}, {-54.6667, 47.6667}, {-54.6667, 41.6667}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(origin = {20, 12}, extent = {{-158, 140}, {158, -140}}), Text(textString = "Power Split", origin = {2, 24}, extent = {{-88, 27}, {88, -27}}), Ellipse(origin = {48, -50}, extent = {{-34.5663198187478, 37.20838371872752}, {34, -38}})}));
  end TransCellsplitter;

  /*FC250kcurrent*/

  block FC250kcurrent
    extends .Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Logical.Switch switch annotation(
      Placement(transformation(extent = {{36.80555555555556, 94.19444444444446}, {56.80555555555556, 114.19444444444446}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(transformation(extent = {{-120.0, 32.0}, {-80.0, 72.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression zVal(y = 0) annotation(
      Placement(transformation(extent = {{-42.0, 86.0}, {-22.0, 106.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.Switch switch2 annotation(
      Placement(transformation(origin = {-4, 0}, extent = {{42, 24}, {62, 44}})));
    Modelica.Blocks.Sources.RealExpression realExpression3 annotation(
      Placement(transformation(extent = {{-22.0, 14.0}, {-2.0, 34.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.LessEqual lessEqual2 annotation(
      Placement(transformation(extent = {{-18.0, -20.0}, {2.0, 0.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression uinput(y = u) annotation(
      Placement(transformation(extent = {{-98.0, 19.165179581113506}, {-78.0, 36.834820418886494}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(transformation(origin = {14, -4}, extent = {{162.475, 12.181}, {182.475, 32.181}}), iconTransformation(extent = {{162.475, 12.181}, {182.475, 32.181}})));
    Modelica.Blocks.Logical.Greater greater annotation(
      Placement(transformation(extent = {{-6.0, 62.0}, {14.0, 82.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.Greater greater2 annotation(
      Placement(transformation(extent = {{-18.0, -46.0}, {2.0, -26.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression zval2(y = 0) annotation(
      Placement(transformation(extent = {{-58.0, -56.0}, {-38.0, -36.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Logical.And and1 annotation(
      Placement(transformation(extent = {{12, -16}, {32, 4}}, origin = {6, -4})));
    Modelica.Blocks.Math.Add add2 annotation(
      Placement(transformation(extent = {{89.7374, 21.0905}, {109.737, 41.0905}})));
    Modelica.Blocks.Sources.RealExpression RequiredPOwer(y = FCPowerRequired) annotation(
      Placement(transformation(extent = {{-12.0, 102.0}, {8.0, 122.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression LowerValue(y = FCLowVal) annotation(
      Placement(transformation(extent = {{-14.111111111111093, 33.11111111111111}, {5.888888888888907, 53.11111111111111}}, origin = {0.0, 0.0}, rotation = 0.0)));
    Modelica.Blocks.Sources.RealExpression I_Control(y = FCMaxRatedCurrent) annotation(
      Placement(transformation(extent = {{88, -6}, {108, 14}}, origin = {-4, -38})));
    Modelica.Blocks.Math.Division division annotation(
      Placement(transformation(extent = {{121.525, 11.819}, {141.525, 31.819}}, origin = {12, -50})));
    Modelica.Blocks.Sources.RealExpression SplitComponent(y = FCLowVal) annotation(
      Placement(transformation(extent = {{-58.0, 6.0}, {-38.0, 26.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
    parameter Modelica.Units.SI.Power FCLowVal;
    parameter Modelica.Units.SI.Power FCPowerRequired;
    parameter Modelica.Units.SI.Power FCMaxRatedPower;
    parameter Modelica.Units.SI.Voltage Stack_v;
    parameter Modelica.Units.SI.Current FCMaxRatedCurrent;
    //Modelica.Blocks.Math.Add add annotation(Placement(transformation(extent = {{102.0,-46.0},{122.0,-26.0}},origin = {0.0,0.0},rotation = 0.0)));
    Modelica.Blocks.Math.Division division1 annotation(
      Placement(transformation(origin = {8, -6}, extent = {{121.525, 11.819}, {141.525, 31.819}})));
    Modelica.Blocks.Sources.RealExpression V_Control(y = Stack_v) annotation(
      Placement(transformation(origin = {94, -6}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(realExpression3.y, switch2.u3) annotation(
      Line(points = {{-1, 24}, {36, 24}, {36, 26}}, color = {0, 0, 127}));
    connect(uinput.y, lessEqual2.u1) annotation(
      Line(points = {{-77, 28}, {-62, 28}, {-62, -10}, {-20, -10}}, color = {0, 0, 127}));
    connect(uinput.y, greater.u1) annotation(
      Line(points = {{-77, 28}, {-42.5, 28}, {-42.5, 72}, {-8, 72}}, color = {0, 0, 127}));
    connect(greater.y, switch.u2) annotation(
      Line(points = {{15, 72}, {24.90277777777778, 72}, {24.90277777777778, 104.19444444444446}, {34.80555555555556, 104.19444444444446}}, color = {255, 0, 255}));
    connect(uinput.y, greater2.u1) annotation(
      Line(points = {{-77, 28}, {-68, 28}, {-68, -36}, {-20, -36}}, color = {0, 0, 127}));
    connect(greater2.y, and1.u2) annotation(
      Line(points = {{3, -36}, {10, -36}, {10, -18}, {16, -18}}, color = {255, 0, 255}));
    connect(lessEqual2.y, and1.u1) annotation(
      Line(points = {{3, -10}, {16, -10}}, color = {255, 0, 255}));
    connect(and1.y, switch2.u2) annotation(
      Line(points = {{39, -10}, {39, 34}, {36, 34}}, color = {255, 0, 255}));
    connect(switch.y, add2.u1) annotation(
      Line(points = {{57.8056, 104.194}, {80, 104.194}, {80, 37.0905}, {87.7374, 37.0905}}, color = {0, 0, 127}));
    connect(I_Control.y, division.u2) annotation(
      Line(points = {{105, -34}, {132, -34}}, color = {0, 0, 127}));
    connect(SplitComponent.y, greater.u2) annotation(
      Line(points = {{-37, 16}, {-32, 16}, {-32, 64}, {-8, 64}}, color = {0, 0, 127}));
    connect(SplitComponent.y, lessEqual2.u2) annotation(
      Line(points = {{-37, 16}, {-32, 16}, {-32, -18}, {-20, -18}}, color = {0, 0, 127}));
    connect(LowerValue.y, switch2.u1) annotation(
      Line(points = {{6.88889, 43.1111}, {12.8889, 43.1111}, {12.8889, 47.1111}, {36, 47.1111}, {36, 42}}, color = {0, 0, 127}));
    connect(zVal.y, switch.u3) annotation(
      Line(points = {{-21, 96}, {-21, 96.19444444444443}, {34.80555555555556, 96.19444444444443}}, color = {0, 0, 127}));
    connect(zval2.y, greater2.u2) annotation(
      Line(points = {{-37, -46}, {-37, -44}, {-20, -44}}, color = {0, 0, 127}));
    connect(RequiredPOwer.y, switch.u1) annotation(
      Line(points = {{9, 112}, {34.80555555555556, 112}, {34.80555555555556, 112.19444444444446}}, color = {0, 0, 127}));
    connect(switch2.y, add2.u2) annotation(
      Line(points = {{59, 34}, {95.7374, 34}, {95.7374, 25.0905}, {87.7374, 25.0905}}, color = {0, 0, 127}));
    connect(add2.y, division1.u1) annotation(
      Line(points = {{110, 32}, {118, 32}, {118, 22}, {128, 22}}, color = {0, 0, 127}));
    connect(V_Control.y, division1.u2) annotation(
      Line(points = {{106, -6}, {110, -6}, {110, 10}, {128, 10}}, color = {0, 0, 127}));
    connect(division1.y, division.u1) annotation(
      Line(points = {{150, 16}, {158, 16}, {158, -6}, {122, -6}, {122, -22}, {132, -22}}, color = {0, 0, 127}));
    connect(division.y, y) annotation(
      Line(points = {{154, -28}, {168, -28}, {168, 18}, {186, 18}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(origin = {32, 2}, extent = {{-138.14115652083177, 120.1959149555095}, {138, -120}}), Text(textString = "FC Logic", origin = {38, 51}, extent = {{-95.77977812132227, 31.868838925895844}, {96, -31}}), Ellipse(origin = {48, -50}, extent = {{-34.5663198187478, 37.20838371872752}, {34, -38}})}),
      Diagram);
  end FC250kcurrent;

  /*FuelCellEffciency*/

  model FuelCellEfficiency
    partial model Base
      //External Model repository for Local Fuel Cell base use
      /*FluidPort*/

      connector FluidPort "Fluid port for low fidelity models in microgrid components"
        replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Hydrogen constrainedby AZEAT_FortunaCrane_Short_ControllerGroupFuel.Templates.SimpleFuel "Transported medium";
        Modelica.Units.SI.Pressure p "Pressure";
        flow Modelica.Units.SI.MassFlowRate m_flow "Mass flow rate";
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Ellipse(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end FluidPort;

      /*FuelCellIcon*/

      model FuelCellIcon
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, radius = 20), Line(points = {{-24, 38}}, color = {28, 108, 200}, thickness = 1), Rectangle(extent = {{-74, 1.05}, {-62, -4.95}}, lineColor = {28, 108, 200}, fillColor = {255, 170, 170}, fillPattern = FillPattern.Solid, origin = {18, 4.95}), Rectangle(extent = {{-10, 1.05}, {1.99999, -4.95}}, lineColor = {28, 108, 200}, fillColor = {170, 213, 255}, fillPattern = FillPattern.Solid, origin = {-6, 4.95}), Polygon(points = {{-44, -6}, {36, -6}, {36, -86}, {-44, -86}, {-44, -6}}, lineColor = {28, 108, 200}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid, origin = {-26, 6}), Text(extent = {{-90, 34}, {-70, 14}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, textString = "+", origin = {30, -34}), Text(extent = {{-2, 34}, {18, 14}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, origin = {-18, -32}, textString = "-"), Line(points = {{-52, -2}, {-52, -10}}, pattern = LinePattern.None), Polygon(points = {{-22, -108}, {-22, -108}}, lineColor = {28, 108, 200}, lineThickness = 0.5, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Line(points = {{-40, 60}}, color = {28, 108, 200}, thickness = 0.5), Line(points = {{-70, -40}, {-80, -40}, {-80, 0}, {-90, 0}}, color = {28, 108, 200}), Polygon(points = {{-32, -2}, {-32, -20}, {-40, -20}, {-40, -60}, {-32, -60}, {-32, -78}, {-28, -78}, {-28, -60}, {-20, -60}, {-20, -20}, {-28, -20}, {-28, -2}, {-32, -2}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{30, 22}, {80, -28}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Line(points = {{38, 14}, {48, 14}}, color = {28, 108, 200}, thickness = 0.5), Line(points = {{38, 12}, {42, 12}}, color = {28, 108, 200}, thickness = 0.5), Line(points = {{44, 12}, {48, 12}}, color = {28, 108, 200}, thickness = 0.5), Line(points = {{62, -16}, {72, -16}}, color = {28, 108, 200}, thickness = 0.5), Line(points = {{68, -18}, {72, -18}}, color = {28, 108, 200}, thickness = 0.5), Line(points = {{62, -18}, {66, -18}}, color = {28, 108, 200}, thickness = 0.5), Line(points = {{30, -28}, {80, 22}}, color = {28, 108, 200}), Line(points = {{-100, 60}, {54, 60}, {54, 22}}, color = {28, 108, 200}), Line(points = {{80, 0}, {90, 0}}, color = {28, 108, 200}), Line(points = {{-10, 6}, {-10, 26}, {20, 26}, {20, 0}, {30, 0}}, color = {28, 108, 200}), Line(points = {{-50, 6}, {-50, 26}, {-10, 26}, {-10, 26}}, color = {28, 108, 200})}));
      end FuelCellIcon;

      /*Economy Thingy*/
      type Time_yr = Real(final quantity = "Time", final unit = "yr");
      type PerPower = Real(final quantity = "PerPower");
      type PerPower_PerkW = PerPower(final unit = "1/kW");
      type PerPowerTime = Real(final quantity = "perPowerTime");
      type PerPowerTime_PerkWyr = PerPowerTime(final unit = "1/(kW.yr)");
      // End of the Repository
      replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Hydrogen constrainedby AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.Hydrogen "Transported medium" annotation(
         choicesAllMatching);
      extends FuelCellIcon;
      //   Medium medium;
      // Parameters
      parameter Modelica.Units.SI.Temperature T = 313.15 "Cell operating temperature" annotation(
        Dialog(group = "Operating point"));
      parameter Modelica.Units.SI.PartialPressure p_H2 = 40000 "Partial pressure of H2 on anode surface of membrane" annotation(
        Dialog(group = "Operating point"));
      parameter Modelica.Units.SI.PartialPressure p_O2 = 21000 "Partial pressure of O2 on cathode surface of membrane" annotation(
        Dialog(group = "Operating point"));
      parameter Modelica.Units.SI.Area A_cell = 180e-4 "Cell area" annotation(
        Dialog(group = "Cell characteristics and polarization"));
      parameter Real n_cell(min = n_cell_min, max = n_cell_max) = 100 "Number of cells in stack, can be used for scaling purposes" annotation(
        Dialog(group = "Stack parametrization"));
      // Advanced parameters
      parameter Modelica.Units.SI.Voltage V_limit = 0.1 "Smoothing interval for voltage to avoid negative voltages. Activates when V < V_limit" annotation(
        Dialog(tab = "Numerics"));
      // Calculated parameters
      final parameter Modelica.Units.SI.Voltage V0 = V0_liquid - 0.85e-3*(T - T_ref25C) + 4.3085e-5*T*(Modelica.Math.log(p_H2/p_NTP) + 0.5*Modelica.Math.log(p_O2/p_NTP)) "Open Circuit Voltage" annotation(
        Dialog(group = "Calculated parameters"));
      // Constants
      constant Modelica.Units.SI.Voltage V0_liquid = 1.229 "Standard potential for lower heat value";
      constant Integer z = 2 "Number of exchanged electrons";
      constant Modelica.Units.SI.Temperature T_ref25C = 298.15 "Reference temperature at 25C";
      constant Modelica.Units.SI.Pressure p_NTP = 101325 "Normal pressure in Pa = 1 atm";
      parameter Modelica.Units.SI.Power P_rat(displayUnit = "kW") = 212*n_cell "Rated power [W]. Corresponds to 0.75 V/cell, which is when concentration losses start to become significant" annotation(
        Dialog(tab = "Economy"));
      // Optimization
      parameter Boolean n_cell_free_ = false "If true, then number of cells is free in optimization" annotation(
        Dialog(group = "Design", tab = "Optimization"));
      parameter Real n_cell_min = 1 "Minimum amount of cells in optimization" annotation(
        Dialog(enable = n_cell_free_, group = "Design", tab = "Optimization"));
      parameter Real n_cell_max = 1000 "Maximum amount of cell in  optimization" annotation(
        Dialog(enable = n_cell_free_, group = "Design", tab = "Optimization"));
      // Control
      parameter Boolean control_power = false "If true, control power. If false control current" annotation(
        Dialog(group = "Control"));
      parameter Modelica.Units.SI.CurrentDensity J_max = 15000 "Maximum current density, when the concentration loss starts getting significant" annotation(
        Dialog(enable = not control_power, group = "Control"));
      parameter Modelica.Units.SI.Current I_max = J_max*A_cell "Maximum current signal" annotation(
        Dialog(enable = not control_power, group = "Control"));
      parameter Modelica.Units.SI.Current I_min = 0 "Minimum current signal" annotation(
        Dialog(enable = not control_power, group = "Control"));
      parameter Modelica.Units.SI.Power P_max = P_rat "Maximum power signal" annotation(
        Dialog(enable = control_power, group = "Control"));
      parameter Modelica.Units.SI.Power P_min = 0 "Minimum power signal" annotation(
        Dialog(enable = control_power, group = "Control"));
      // Variables
      Modelica.Units.SI.Voltage V_stack "Stack Voltage";
      Modelica.Units.SI.Voltage V_cell "Cell Voltage";
      Modelica.Units.SI.CurrentDensity J "Current density";
      Modelica.Units.SI.Current I "Stack current";
      Modelica.Units.SI.MolarFlowRate ndot_H2 "Hyrdogen molar flow rate";
      Modelica.Units.SI.HeatFlowRate Q_cool "Cooling demand";
      // Ideal transformer
      Modelica.Units.SI.Current I_out "Current at secondary side of Ideal transformer";
      Modelica.Units.SI.Voltage V_out "Voltage at secondary side of Ideal transformer";
      // Summary variables
      Modelica.Units.SI.Power P_stack "Stack total power output";
      Modelica.Units.SI.Power P_loss "Stack total power loss (Based on LHV)";
      Modelica.Units.SI.Efficiency eta_e "Electrical efficiency";
      // Connector Interface
      Modelica.Electrical.Analog.Interfaces.NegativePin pin "Electrical Connector" annotation(
        Placement(transformation(extent = {{86.93043192491228, -13.069568075087687}, {113.06956807508772, 13.069568075087687}}, rotation = 0.0, origin = {0.0, 0.0}), iconTransformation(extent = {{90, -10}, {110, 10}})));
      FluidPort fluidPort(redeclare package Medium = Medium) "Hydrogen flow Connector" annotation(
        Placement(transformation(extent = {{-110.0, -10.0}, {-90.0, 10.0}}, origin = {0.0, 0.0}, rotation = 0.0)));
      Modelica.Blocks.Interfaces.RealInput I_control if not control_power "Current control input in p.u." annotation(
        Placement(transformation(extent = {{-20.0, -20.0}, {20.0, 20.0}}, origin = {0.0, -120.0}, rotation = 90.0), iconTransformation(extent = {{-12, -12}, {12, 12}}, rotation = 0, origin = {-112, 60})));
      Modelica.Blocks.Interfaces.RealInput P_control if control_power "Power control input in p.u." annotation(
        Placement(transformation(extent = {{-20.0, -20.0}, {20.0, 20.0}}, origin = {0, -120}, rotation = 90.0), iconTransformation(extent = {{-12, -12}, {12, 12}}, rotation = 0, origin = {-112, 60})));
    protected
      Modelica.Blocks.Interfaces.RealInput u_ctrl "Local control interface";
      //To allow switching between controlled current and power
    equation
// Electrical connection
      I = A_cell*J;
// Control
      if control_power then
        P_stack = u_ctrl*(P_max - P_min) + P_min;
      else
        I = u_ctrl*(I_max - I_min) + I_min;
      end if;
// Ideal transformer
      I*V_stack = I_out*V_out;
      pin.i = -I_out;
      pin.v = V_out;
// Hydrogen consumption
      ndot_H2 = I*n_cell/(z*Modelica.Constants.F);
// Hydrogen connection
      fluidPort.m_flow = ndot_H2*Medium.MolarMass;
// Heat generated
      Q_cool = n_cell*V0*I - P_stack;
// Summary
      V_stack = V_cell*n_cell;
      P_stack = I*V_stack;
      n_cell*V0*eta_e = V_stack;
      P_loss = fluidPort.m_flow*Medium.LHV - P_stack;
      connect(P_control, u_ctrl);
      connect(I_control, u_ctrl);
      annotation(
        Dialog(tab = "Economy"));
    end Base;

    extends Base(P_max = n_cell*V_cell_max*I_max);
    parameter Real eta = 0.6 "Fuel cell efficiency" annotation(
      Dialog(group = "Stack parametrization"));
    final parameter Modelica.Units.SI.Voltage V_cell_max = V0*eta;
  equation
    V_cell = V0*eta;
    annotation(
      experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
      __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=NLSanalyticJacobian",
      __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
  end FuelCellEfficiency;

  /*DCBus*/

  model DCbusBar "DC Busbar"
    output Modelica.Units.SI.Voltage v(stateSelect = StateSelect.never);
    Modelica.Electrical.Analog.Interfaces.PositivePin term "Terminal" annotation(
      Placement(transformation(extent = {{-7, -60}, {7, 60}}, rotation = 0)));
  equation
    term.i = 0;
    v = term.v;
    annotation(
      defaultComponentName = "bus1",
      Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics = {Text(extent = {{-100, -98}, {100, -120}}, lineColor = {0, 0, 0}, textString = "%name"), Rectangle(extent = {{-10, 80}, {10, -80}}, lineColor = {0, 0, 255}, pattern = LinePattern.None, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid)}));
  end DCbusBar;

  // Repository End MasterModel
  parameter Modelica.Units.SI.Power P_nominal = 1e9 "Nominal power in the micro grid";
  parameter Boolean optim = false "True, if model is optimized, dynamic simulation if false";
  parameter Integer N_gen = 3 "Number of Generator";
  parameter Integer N_alt_gen = 3 "Number of  Alternative Generator ";
  parameter Integer N_diesel_gen_onboard = 2  "Number for diesel generator for case study";
  parameter Integer N_alt_fuel_gen_onboard = 0 "Number  for alternative fuel  generator  for case study ";
  parameter Real H2SOCstart = 0;
  parameter Modelica.Units.SI.Volume H2Tankstorage = 23;
  /*Component Parameters for API  Exposure*/
  parameter Real transCellSplitter_MaxRatedPowerCell;
  parameter Real transCellSplitter_Minimal_SOC;
  parameter Real fc250CurrentLogic_LowOperationValue = 50000;
  parameter Real fc250CurrentLogic_HighOperationValue = 100000;
  parameter Modelica.Units.SI.Volume hydrogenTank_Volume = H2Tankstorage;
  parameter Real hydrogenTank_Tempurature;
  parameter Boolean hydrogenTank_use_pressure_constraint = true;
  parameter Real fuelCell_Tempurature;
  //parameter Fuel fuelCell_Medium
  parameter Real fuelCell_P_H2;
  parameter Real fuelCell_P_Q2;
  parameter Real fuelCell_P_max = 100000;
  parameter Real fuelCell_P_min;
  parameter Real fuelCell_A_cell = 0.03;
  parameter Real fuelCell_n_cell = 300;
  parameter Real fuelCell_eta = 0.4;
  parameter Boolean fuelCell_control_power = false;
  parameter Real mCtrl_SOC_tank_Sec = 0.1;
  parameter Real mCtrl_SOC_min_Sec = 0.1;
  parameter Real mCtrl_DieselControlforBattery = 0;
  parameter Real mCtrl_SOC_max = 0.9;
  parameter Real mCtrl_SOC_min = 0.2;
  parameter Real mCtrl_PiroritySchemeAssignement = 0;
  //{choose between ICE or Fuel Cell or Battery First }
  parameter Real mCtrl_IcePriorityAssignement = 0;
  // {Within all ICE, choose Diesel or Methanol First to meed the demand}
  parameter Real mCtrl_IceChargingPriorityAssignement = 0;
  // {Within all ICE , choose who charge the battery}
  parameter Real mCtrl_dieselGenPercentage = 0.5;
  parameter Real mCtrl_fuelCellPercentage = 1 - mCtrl_dieselGenPercentage;
  // parameter Diesel Fuel-Generator_Fuel
  parameter Modelica.Units.SI.SpecificEnergy generator_FLHV = 45.9e6 "Fuel LHV";
  parameter Modelica.Units.SI.Density generator_Frho = 846 "Fuel Density KG/M^3";
  parameter Real generator_Frho_liq = 846 "Fuel Liquid Density";
  parameter Real generator_FcarbonContent = 0.86 "Fuel Carbon Content";
  parameter Modelica.Units.SI.MolarMass generator_MolarMass = 0.233 "Fuel MolarMass";
  parameter Real generator_M_Flow_Fidle = 0.00008*((0.08451*800));
  parameter Real generator_M_Flow_FRat = 0.00027777777*((0.08451*800));
  parameter Real generator_V_flow_Fidle = 1.02e-5;
  parameter Real generator_V_flow_Frat = 5.88e-5;
  parameter Boolean generator_use_fuel_eff_table = true;
  //parameter Alt Fuel Fuel Generator Selector
  parameter Modelica.Units.SI.SpecificEnergy generator_alt_FLHV = 23e6 "Fuel LHV";
  parameter Modelica.Units.SI.Density generator_alt_Frho = 791 "Fuel Density KG/M^3";
  parameter Real generator_alt_Frho_liq = 791 "Fuel Liquid Density";
  parameter Real generator_alt_FcarbonContent = 0.74 "Fuel Carbon Content";
  parameter Modelica.Units.SI.MolarMass generator_alt_MolarMass = 0.233 "Fuel MolarMass";
  parameter Real generator_alt_M_Flow_Fidle = 0.00008*((0.08451*800));
  parameter Real generator_alt_M_Flow_FRat = 0.00027777777*((0.08451*800));
  parameter Real generator_alt_V_flow_Fidle = 1.22e-5;
  parameter Real generator_alt_V_flow_Frat = 5.21e-5;
  parameter Boolean generator_alt_fuel_eff_table = true;
  //Diesel Generator Defination Parameter
  parameter Boolean generator_Use_Const_Off = false;
  parameter Real generator_Eta_El_Const = 0.40;
  parameter Boolean generator_FuelConsumptionIsVolumetric = true;
  parameter Real generator_P_rat = 800000;
  parameter Real generator_P_rat_1 = 800000;
  parameter Real generator_P_rat_2 = 800000;
  parameter Real generator_P_rat_3 = 800000;
  parameter Real generator_P_charging = 50000;
  parameter Real generator_P_charging_1 = 50000;
  parameter Real generator_P_charging_2 = 50000;
  parameter Real generator_P_charging_3 = 50000;
  parameter Real generator_P_idle = 100000;
  parameter Real generator_P_idle_1 = 100000;
  parameter Real generator_P_idle_2 = 100000;
  parameter Real generator_P_idle_3 = 100000;
  //Alt Fuel Generator Defination Parameter
  parameter Real generator_alt_P_rat = 564000;
  parameter Real generator_alt_P_rat_1 = 564000;
  parameter Real generator_alt_P_rat_2 = 564000;
  parameter Real generator_alt_P_rat_3 = 564000;
  parameter Real generator_alt_P_charging = 50000;
  parameter Real generator_alt_P_charging_1 = 50000;
  parameter Real generator_alt_P_charging_2 = 50000;
  parameter Real generator_alt_P_charging_3 = 50000;
  parameter Real generator_alt_P_idle = 100000;
  parameter Real generator_alt_P_idle_1 = 100000;
  parameter Real generator_alt_P_idle_2 = 100000;
  parameter Real generator_alt_P_idle_3 = 100000;
  parameter Real generator_actual_max_total = generator_P_rat*N_diesel_gen_onboard;
  parameter Real alt_generatoractual_max_total = generator_alt_P_rat*N_alt_fuel_gen_onboard;
  parameter Boolean diesel_gen1_is_on = true;
  parameter Boolean diesel_gen2_is_on = true;
  parameter Boolean diesel_gen3_is_on = false;
  parameter Boolean diesel_gen4_is_on = false;
  parameter Boolean diesel_gen5_is_on = false;
  parameter Boolean altFuel_gen1_is_on = false;
  parameter Boolean altFuel_gen2_is_on = false;
  parameter Boolean altFuel_gen3_is_on = false;
  parameter Boolean electricalLoad_Use_Input = true;
  parameter Real electricalLoad_Load_In = 1000;
  parameter Real transformer_Effciency = 1;
  parameter Modelica.Units.SI.Power transformer_Power_Nominal = P_nominal;
  parameter Modelica.Units.SI.Power transformer_P_max = 10*P_nominal;
  parameter Real battery_Eff_Charge = 0.99;
  parameter Boolean battery_Set_SOC_Final_Start = true;
  parameter Real battery_Capacity = 112680000 *  10 ;
  parameter Real battery_P_max = 46620 * 10  ; // 46620
  parameter Real battery_SOC_start = 0.85;
  parameter Modelica.Units.SI.Power electricalGrid_Power_Nominal = P_nominal;
  parameter Real electricalGrid_V_ref = 1000;
  parameter Modelica.Units.SI.Mass idealTank_TankcontentDiesel = 2e14;
  parameter Modelica.Units.SI.Mass idealTank_TankcontentDieselStart = 2e14;
  parameter Modelica.Units.SI.Mass idealTank_TankcontentAltFuel = 2e14;
  parameter Modelica.Units.SI.Mass idealTank_TankcontentAltFuelStart = 2e14;
  //750, 100
  parameter Boolean mCtrl_user_defined_Bounds_diesel = false;
  parameter Real mCtrl_user_defined_upper_bound_diesel = 750;
  parameter Real mCtrl_user_defined_lower_bound_diesel = 700;
  parameter Boolean mCtrl_user_defined_Bounds_alt_fuel = false;
  parameter Real mCtrl_user_defined_upper_bound_alt_fuel = 400;
  parameter Real mCtrl_user_defined_lower_bound_alt_fuel = 200;
  //[80, 180.80; 160, 172.11; 240, 167.23; 320, 163.85; 400, 161.27; 480, 159.20; 560, 157.47; 640, 155.98; 720, 154.68; 800, 153.53] old
  parameter Real BSFC_Curve[:, 2] =  [80, 197.91; 160, 189.97; 240, 183.23; 320, 177.71; 400, 173.40; 480, 170.30; 560, 168.42; 640, 167.74; 720, 168.28; 800, 170.03];
  parameter Real BSFC_Curve_AltFuel[:, 2] = [145.9375, 219.7885196; 153.125, 216.9939577; 164.0625, 213.0664653; 188.75, 206.0422961; 211.25, 201.2084592; 232.1875, 196.978852; 251.875, 194.0332326; 269.6875, 192.3716012; 282.5, 191.6918429; 302.8125, 192.5226586; 318.125, 192.8247734; 340.625, 193.9577039; 360, 195.3172205; 380.3125, 196.9033233; 409.6875, 199.244713; 445.625, 202.1903323; 468.4375, 204.0030211; 495.625, 206.9486405; 529.6875, 211.6314199; 555.9375, 215.4833837; 564.375, 216.8429003];

//[0, 0.00; 10, 16.74; 20, 31.87; 30, 46.45; 40, 60.68; 50, 74.66; 60, 88.44; 70, 102.06; 80, 115.54; 90, 128.90; 100, 142.16] old
  parameter Real Engine_Fuel_Consumption_Look_Up_Table_Diesle[:, 2] =[0, 0.00; 10, 18.72; 20, 35.93; 30, 51.98; 40, 67.22; 50, 81.99; 60, 96.63; 70, 111.48; 80, 126.90; 90, 143.22; 100, 160.79];

  parameter Real Engine_Fuel_Consumption_Look_Up_Table_Diesle_1[:, 2] = [0, 0.00; 10, 19.93; 20, 37.94; 30, 55.30; 40, 72.24; 50, 88.89; 60, 105.29; 70, 121.50; 80, 137.55; 90, 153.45; 100, 169.23];
  parameter Real Engine_Fuel_Consumption_Look_Up_Table_Diesle_2[:, 2] = [0, 20.35; 10, 38.75; 20, 56.48; 30, 73.78; 40, 90.78; 50, 107.53; 60, 124.09; 70, 140.48; 80, 156.72; 90, 172.83; 100, 182.77];
  parameter Real Engine_Fuel_Consumption_Look_Up_Table_Diesle_3[:, 2] = [0, 20.35; 10, 38.75; 20, 56.48; 30, 73.78; 40, 90.78; 50, 107.53; 60, 124.09; 70, 140.48; 80, 156.72; 90, 172.83; 100, 182.77];
  parameter Real Engine_Fuel_Consumption_Look_Up_Table_AltFuel[:, 2] = [0, 0; 10, 24.93; 20, 47.46; 30, 69.17; 40, 90.37; 50, 111.18; 60, 131.70; 70, 151.98; 80, 172.05; 90, 191.95; 100, 211.68];
  parameter Real Engine_Fuel_Consumption_Look_Up_Table_AltFuel_1[:, 2] = [0, 0; 10, 24.93; 20, 47.46; 30, 69.17; 40, 90.37; 50, 111.18; 60, 131.70; 70, 151.98; 80, 172.05; 90, 191.95; 100, 211.68];
  parameter Real Engine_Fuel_Consumption_Look_Up_Table_AltFuel_2[:, 2] = [0, 0; 10, 24.93; 20, 47.46; 30, 69.17; 40, 90.37; 50, 111.18; 60, 131.70; 70, 151.98; 80, 172.05; 90, 191.95; 100, 211.68];
  parameter Real Engine_Fuel_Consumption_Look_Up_Table_AltFuel_3[:, 2] = [0, 0; 10, 24.93; 20, 47.46; 30, 69.17; 40, 90.37; 50, 111.18; 60, 131.70; 70, 151.98; 80, 172.05; 90, 191.95; 100, 211.68];
  //[0,20.35; 10,38.75; 20,56.48; 30,73.78; 40,90.78; 50,107.53; 60,124.09; 70,140.48; 80,156.72; 90,172.83; 100,182.77]
  //[0, 0; 10, 24.93; 20, 47.46; 30, 69.17; 40, 90.37;50, 111.18; 60, 131.70; 70, 151.98; 80, 172.05; 90, 191.95;100,211.68]
  /*END--Component Parameters for API  Exposure*/
  /*Master Model Parameters and  Component Instantiation*/
  // Part Battery
  AZEAT_FortunaCrane_Short_ControllerGroup.BatteryDualControl battery1(
          eff_charge = battery_Eff_Charge, 
          set_SOC_final_start_ = battery_Set_SOC_Final_Start, 
          SOC_max = mCtrl_SOC_max, SOC_min = mCtrl_SOC_min, 
          capacity = battery_Capacity, 
          P_max = battery_P_max, 
          SOC_start = battery_SOC_start) annotation(
    Placement(transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}})));
  // Part Electrical Untitlity
  AZEAT_FortunaCrane_Short_ControllerGroup.Converter_ACDC converter_ACDC(efficiency = 1, V_ref_DC = 1, power_nominal = P_nominal, P_max = 10*P_nominal) "Inverter" annotation(
    Placement(transformation(origin = {108, 54}, extent = {{-10, -10}, {10, 10}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.ElectricalGrid electricalGrid(power_nominal = P_nominal, V_ref = 1000) annotation(
    Placement(transformation(origin = {156, 52}, extent = {{-10, -10}, {10, 10}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.ElectricLoad electricLoad(use_input = true, load_in = 1000) annotation(
    Placement(transformation(origin = {71, 29}, extent = {{11, -11}, {-11, 11}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.Transformer transformer(efficiency = 1, power_nominal = P_nominal, P_max = 10*P_nominal) annotation(
    Placement(transformation(origin = {30, 46}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Sources.Sine electric_purchasePrice(phase = -0.5*.Modelica.Constants.pi, f = 1/(72*3600), amplitude = 0.08*0.4, offset = 0.08) annotation(
    Placement(transformation(origin = {196, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  //Part real Expression
  Modelica.Blocks.Sources.RealExpression BattSOC(y = battery1.SOC) if not optim annotation(
    Placement(transformation(origin = {-130, 94}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression LoadPower(y = electricLoad.P_load) if not optim annotation(
    Placement(transformation(origin = {-130, 62}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression RenewablePower(y = fuelCell2.P_stack + fuelCell3.P_stack) annotation(
    Placement(transformation(origin = {-130, 78}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression TotalGeneratorSOC(y = idealTankMultiPort.SOC) if not optim annotation(
    Placement(transformation(origin = {-60, 164}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
  // Part Mechanical
  AZEAT_FortunaCrane_Short_ControllerGroup.Generator generator1(
    power_vs_fuelconsum = Engine_Fuel_Consumption_Look_Up_Table_Diesle, 
    V_flow_frat = generator_V_flow_Frat, 
    V_flow_fidle = generator_V_flow_Fidle, 
    use_const_eff = generator_Use_Const_Off, 
    eta_el_const = generator_Eta_El_Const, 
    fuelConsumptionIsVolumetric = true, 
    m_flow_fidle = generator_M_Flow_Fidle, 
    m_flow_frat = generator_M_Flow_FRat, 
    P_rat = generator_P_rat, 
    FLHV = generator_FLHV, 
    Frho = generator_Frho, 
    Frho_liq = generator_Frho_liq, 
    FcarbonContent = generator_FcarbonContent, FMolarMass = generator_MolarMass, use_fuel_eff_table = generator_use_fuel_eff_table) annotation(
    Placement(transformation(origin = {-10, 10}, extent = {{10, -10}, {-10, 10}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.Generator generator2(power_vs_fuelconsum = Engine_Fuel_Consumption_Look_Up_Table_Diesle, V_flow_frat = generator_V_flow_Frat, V_flow_fidle = generator_V_flow_Fidle, use_const_eff = generator_Use_Const_Off, eta_el_const = generator_Eta_El_Const, fuelConsumptionIsVolumetric = true, m_flow_fidle = generator_M_Flow_Fidle, m_flow_frat = generator_M_Flow_FRat, P_rat = generator_P_rat, FLHV = generator_FLHV, Frho = generator_Frho, Frho_liq = generator_Frho_liq, FcarbonContent = generator_FcarbonContent, FMolarMass = generator_MolarMass, use_fuel_eff_table = generator_use_fuel_eff_table) annotation(
    Placement(transformation(origin = {-10, -25}, extent = {{10, -9}, {-10, 9}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.Generator generator3(power_vs_fuelconsum = Engine_Fuel_Consumption_Look_Up_Table_Diesle, V_flow_frat = generator_V_flow_Frat, V_flow_fidle = generator_V_flow_Fidle, use_const_eff = generator_Use_Const_Off, eta_el_const = generator_Eta_El_Const, fuelConsumptionIsVolumetric = true, m_flow_fidle = generator_M_Flow_Fidle, m_flow_frat = generator_M_Flow_FRat, P_rat = generator_P_rat, FLHV = generator_FLHV, Frho = generator_Frho, Frho_liq = generator_Frho_liq, FcarbonContent = generator_FcarbonContent, FMolarMass = generator_MolarMass, use_fuel_eff_table = generator_use_fuel_eff_table) annotation(
    Placement(transformation(origin = {-10, -56}, extent = {{10, -10}, {-10, 10}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.Generator generator_af_1(power_vs_fuelconsum = Engine_Fuel_Consumption_Look_Up_Table_AltFuel, V_flow_frat = generator_alt_V_flow_Frat, V_flow_fidle = generator_alt_V_flow_Fidle, use_const_eff = generator_Use_Const_Off, use_fuel_eff_table = generator_alt_fuel_eff_table, eta_el_const = generator_Eta_El_Const, fuelConsumptionIsVolumetric = true, m_flow_fidle = generator_alt_M_Flow_Fidle, m_flow_frat = generator_alt_M_Flow_FRat, P_rat = generator_alt_P_rat, FLHV = generator_alt_FLHV, Frho = generator_alt_Frho, Frho_liq = generator_alt_Frho_liq, FcarbonContent = generator_alt_FcarbonContent, FMolarMass = generator_alt_MolarMass) annotation(
    Placement(transformation(origin = {-10, -90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  AZEAT_FortunaCrane_Short_ControllerGroup.Generator generator_af_2(power_vs_fuelconsum = Engine_Fuel_Consumption_Look_Up_Table_AltFuel, V_flow_frat = generator_alt_V_flow_Frat, V_flow_fidle = generator_alt_V_flow_Fidle, use_const_eff = generator_Use_Const_Off, eta_el_const = generator_Eta_El_Const, use_fuel_eff_table = generator_alt_fuel_eff_table, fuelConsumptionIsVolumetric = true, m_flow_fidle = generator_alt_M_Flow_Fidle, m_flow_frat = generator_alt_M_Flow_FRat, P_rat = generator_alt_P_rat, FLHV = generator_alt_FLHV, Frho = generator_alt_Frho, Frho_liq = generator_alt_Frho_liq, FcarbonContent = generator_alt_FcarbonContent, FMolarMass = generator_alt_MolarMass) annotation(
    Placement(transformation(origin = {-10, -130}, extent = {{10, -10}, {-10, 10}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.Generator generator_af_3(power_vs_fuelconsum = Engine_Fuel_Consumption_Look_Up_Table_AltFuel, V_flow_frat = generator_alt_V_flow_Frat, V_flow_fidle = generator_alt_V_flow_Fidle, use_const_eff = generator_Use_Const_Off, eta_el_const = generator_Eta_El_Const, use_fuel_eff_table = generator_alt_fuel_eff_table, fuelConsumptionIsVolumetric = true, m_flow_fidle = generator_alt_M_Flow_Fidle, m_flow_frat = generator_alt_M_Flow_FRat, P_rat = generator_alt_P_rat, FLHV = generator_alt_FLHV, Frho = generator_alt_Frho, Frho_liq = generator_alt_Frho_liq, FcarbonContent = generator_alt_FcarbonContent, FMolarMass = generator_alt_MolarMass) annotation(
    Placement(transformation(origin = {-10, -170}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  AZEAT_FortunaCrane_Short_ControllerGroup.IdealTankMultiPort idealTankMultiPort(m_content_max = idealTank_TankcontentDiesel, m_content_start = idealTank_TankcontentDieselStart, redeclare replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.MethanolFuel) annotation(
    Placement(transformation(origin = {70, -28}, extent = {{-10, -10}, {10, 10}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.IdealTankMultiPort alt_fuel_tank(m_content_max = idealTank_TankcontentAltFuel, m_content_start = idealTank_TankcontentAltFuelStart, redeclare replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.MethanolFuel) annotation(
    Placement(transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}})));
  // Part Renewable
  TransCellsplitter dualCellSplitter annotation(
    Placement(transformation(origin = {-6, 128}, extent = {{-10, -10}, {10, 10}})));
  FC250kcurrent fC250kcurrent2(FCLowVal = fc250CurrentLogic_LowOperationValue, FCPowerRequired = fc250CurrentLogic_HighOperationValue, FCMaxRatedPower = fuelCell2.P_max, Stack_v = fuelCell2.V_cell_max*fuelCell2.n_cell, FCMaxRatedCurrent = fuelCell2.I_max) annotation(
    Placement(transformation(origin = {44, 166}, extent = {{-10, -10}, {10, 10}})));
  FC250kcurrent fC250kcurrent3(FCLowVal = fc250CurrentLogic_LowOperationValue, FCPowerRequired = fc250CurrentLogic_HighOperationValue, FCMaxRatedPower = fuelCell2.P_max, Stack_v = fuelCell2.V_cell_max*fuelCell2.n_cell, FCMaxRatedCurrent = fuelCell2.I_max) annotation(
    Placement(transformation(origin = {44, 128}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression TankSOC(y = tank2.SOC) if not optim annotation(
    Placement(transformation(origin = {-28, 166}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.RealExpression TankSOC2(y = tank3.SOC) if not optim annotation(
    Placement(transformation(origin = {-6, 166}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Math.Gain gain1(k = 1000) annotation(
    Placement(transformation(origin = {124, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Tank tank2(redeclare replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.compressedhydrogen, SOC_start = H2SOCstart, V = H2Tankstorage, T(displayUnit = "K"), use_pressure_constraint = true) annotation(
    Placement(transformation(origin = {70, 142}, extent = {{-10, -10}, {10, 10}})));
  Tank tank3(redeclare replaceable package Medium = AZEAT_FortunaCrane_Short_ControllerGroup.Fuel.compressedhydrogen, SOC_start = H2SOCstart, V = H2Tankstorage, T(displayUnit = "K"), use_pressure_constraint = true) annotation(
    Placement(transformation(origin = {72, 86}, extent = {{-10, -10}, {10, 10}})));
  FuelCellEfficiency fuelCell2(A_cell = fuelCell_A_cell, n_cell = fuelCell_n_cell, eta = fuelCell_eta, control_power = fuelCell_control_power, P_max = fuelCell_P_max) annotation(
    Placement(transformation(origin = {110, 170}, extent = {{-10, -10}, {10, 10}})));
  FuelCellEfficiency fuelCell3(A_cell = fuelCell_A_cell, n_cell = fuelCell_n_cell, eta = fuelCell_eta, control_power = fuelCell_control_power, P_max = fuelCell_P_max) annotation(
    Placement(transformation(origin = {110, 110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage2(V = 1) annotation(
    Placement(transformation(origin = {150, 160}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage3(V = 1) annotation(
    Placement(transformation(origin = {150, 110}, extent = {{-10, -10}, {10, 10}})));
  DCbusBar bus3 annotation(
    Placement(transformation(origin = {188, 110}, extent = {{-10, -10}, {10, 10}})));
  DCbusBar bus2 annotation(
    Placement(transformation(origin = {188, 162}, extent = {{-10, -10}, {10, 10}})));
  // Part Controller
  AZEAT_FortunaCrane_Short_ControllerGroup.MasterControllerSingleBattery masterControllerSingleBattery(
    n = N_gen, 
    alt_n = N_alt_gen, 
    acutal_diesel_n = N_diesel_gen_onboard, 
    actual_altFuel_n = N_alt_fuel_gen_onboard, 
    SOC_tank_sec = mCtrl_SOC_tank_Sec, 
    SOC_min_sec = mCtrl_SOC_min_Sec, 
    P_charging_max = battery1.P_max, 
    P_max = {generator1.P_rat, generator2.P_rat, generator3.P_rat}, 
    P_max_alt = {generator_af_1.P_rat, generator_af_2.P_rat, generator_af_3.P_rat}, 
    DieselControlforBattery = mCtrl_DieselControlforBattery, SOC_max = mCtrl_SOC_max, SOC_min = mCtrl_SOC_min, 
    diesel_gen_charge_rate = generator_P_charging, 
    alt_gen_charge_rate = generator_alt_P_charging, 
    battery_rated_gen = battery_P_max, 
    diesel_gen_idling_rate = generator_P_idle, alt_gen_idling_rate = generator_alt_P_idle, P_fuelCellMax_ref = fuelCell3.P_rat, 
    P_rate_combined_ICE_gen = (generator_P_rat*N_diesel_gen_onboard + generator_alt_P_rat*N_alt_fuel_gen_onboard), 
    P_max_onboard = generator_actual_max_total, P_alt_max_onboard = alt_generatoractual_max_total, 
    ParameterAssignement = mCtrl_PiroritySchemeAssignement, IcePriorityAssignement = mCtrl_IcePriorityAssignement, 
    IceChargingPriorityAssignment = mCtrl_IceChargingPriorityAssignement) annotation(
    Placement(transformation(origin = {-60, 47}, extent = {{-18, -17}, {18, 17}})));
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(tableOnFile = true, tableName = "tab1", fileName = "C:/Users/NMS08/Desktop/ReCreatedModel/AZETValidationCase/2024-04_FC_Cardinal_buoy_maintenance.txt", verboseRead = true, columns = 2:2, smoothness = Modelica.Blocks.Types.Smoothness.ConstantSegments, verboseExtrapolation = true) annotation(
    Placement(transformation(origin = {194, 2}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.RealExpression time_expre2(y = time) annotation(
    Placement(transformation(origin = {270, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.RealExpression TotalFuelVolumn annotation(
    Placement(transformation(origin = {130, -130}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integratorCO2 annotation(
    Placement(transformation(origin = {170, -88}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integratorFMass annotation(
    Placement(transformation(origin = {170, -110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integratorFVolumn annotation(
    Placement(transformation(origin = {170, -130}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interaction.Show.RealValue realValueCO2 annotation(
    Placement(transformation(origin = {210, -88}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interaction.Show.RealValue realValueFuelMass annotation(
    Placement(transformation(origin = {210, -110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interaction.Show.RealValue realValueFuelVolumn annotation(
    Placement(transformation(origin = {210, -130}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression InstantFuelConsumption(y = generator1.m_flow_fuel*1000) annotation(
    Placement(transformation(origin = {-220, -116}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interaction.Show.RealValue SpecificFuelConsumption annotation(
    Placement(transformation(origin = {-112, -116}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Division division annotation(
    Placement(transformation(origin = {-186, -122}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression ConstP_Rat(y = 355) annotation(
    Placement(transformation(origin = {-220, -130}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Product product annotation(
    Placement(transformation(origin = {-148, -116}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression ConstSecondPerHour(y = 3600) annotation(
    Placement(transformation(origin = {-186, -98}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression TotalGeneratorCO2(y = (if diesel_gen1_is_on then generator1.m_flow_CO2 else 0) + (if diesel_gen2_is_on then generator2.m_flow_CO2 else 0) + (if diesel_gen3_is_on then generator3.m_flow_CO2 else 0)) annotation(
    Placement(transformation(origin = {130, -88}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression TotalFuelMass(y = (if diesel_gen1_is_on then generator1.outlet_fuel.m_flow else 0) + (if diesel_gen2_is_on then generator2.outlet_fuel.m_flow else 0) + (if diesel_gen3_is_on then generator3.outlet_fuel.m_flow else 0)) annotation(
    Placement(transformation(origin = {130, -110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression AlternativeGeneratorCO2(y = (if altFuel_gen1_is_on then generator_af_1.m_flow_CO2 else 0) + (if altFuel_gen2_is_on then generator_af_2.m_flow_CO2 else 0) + (if altFuel_gen3_is_on then generator_af_3.m_flow_CO2 else 0)) annotation(
    Placement(transformation(origin = {130, -170}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression AlternativeFuelMass(y = (if altFuel_gen1_is_on then generator_af_1.outlet_fuel.m_flow else 0) + (if altFuel_gen2_is_on then generator_af_2.outlet_fuel.m_flow else 0) + (if altFuel_gen3_is_on then generator_af_3.outlet_fuel.m_flow else 0)) annotation(
    Placement(transformation(origin = {130, -190}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression AlternativeFuelVolumn annotation(
    Placement(transformation(origin = {130, -210}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integratorAltCO2 annotation(
    Placement(transformation(origin = {170, -170}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integratorAltFMass annotation(
    Placement(transformation(origin = {170, -190}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integratorAltFVolumn annotation(
    Placement(transformation(origin = {170, -210}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interaction.Show.RealValue realValueAltCO2 annotation(
    Placement(transformation(origin = {210, -170}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interaction.Show.RealValue realValueAltFuelMass annotation(
    Placement(transformation(origin = {210, -190}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interaction.Show.RealValue realValueAltFVolumn annotation(
    Placement(transformation(origin = {210, -210}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression dieselTankSOC_1(y = if diesel_gen1_is_on then idealTankMultiPort.SOC else 0) annotation(
    Placement(transformation(origin = {-170, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression dieselTankSOC_2(y = if diesel_gen2_is_on then idealTankMultiPort.SOC else 0) annotation(
    Placement(transformation(origin = {-150, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression dieselTankSOC_3(y = if diesel_gen3_is_on then idealTankMultiPort.SOC else 0) annotation(
    Placement(transformation(origin = {-130, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression altFuelTankSOC_1(y = if altFuel_gen1_is_on then alt_fuel_tank.SOC else 0) annotation(
    Placement(transformation(origin = {-110, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.RealExpression altFuelTankSOC_2(y = if altFuel_gen2_is_on then alt_fuel_tank.SOC else 0) annotation(
    Placement(transformation(origin = {-90, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.RealExpression altFuelTankSOC_3(y = if altFuel_gen3_is_on then alt_fuel_tank.SOC else 0) annotation(
    Placement(transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  AZEAT_FortunaCrane_Short_ControllerGroup.OptimalFuelConsumptionBoundsCalculator_ExternalTable altFuel_bsfc_table(BSFC_Curve_Default = BSFC_Curve_AltFuel, manually_define_bounds = mCtrl_user_defined_Bounds_alt_fuel, manual_defined_upperBound = mCtrl_user_defined_upper_bound_alt_fuel, manual_defined_lowerBound = mCtrl_user_defined_lower_bound_alt_fuel) annotation(
    Placement(transformation(origin = {-230, 132}, extent = {{10, 10}, {-10, -10}})));
  AZEAT_FortunaCrane_Short_ControllerGroup.OptimalFuelConsumptionBoundsCalculator_ExternalTable diesel_bsfc_table(BSFC_Curve_Default =  BSFC_Curve, manually_define_bounds = mCtrl_user_defined_Bounds_diesel, manual_defined_upperBound = mCtrl_user_defined_upper_bound_diesel, manual_defined_lowerBound = mCtrl_user_defined_lower_bound_diesel) annotation(
    Placement(transformation(origin = {-232, 52}, extent = {{10, 10}, {-10, -10}})));
  Modelica.Blocks.Math.Gain gainAltMinBSFC(k = 1000) annotation(
    Placement(transformation(origin = {-194, 118}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Blocks.Math.Gain gainAltLowerBound(k = 1000) annotation(
    Placement(transformation(origin = {-194, 132}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Blocks.Math.Gain gainAltUpperBound(k = 1000) annotation(
    Placement(transformation(origin = {-194, 144}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Blocks.Math.Gain gainDieselUpperBound(k = 1000) annotation(
    Placement(transformation(origin = {-194, 70}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Blocks.Math.Gain gainDieselLowerBound(k = 1000) annotation(
    Placement(transformation(origin = {-194, 52}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Blocks.Math.Gain gainDieselMinBSFC(k = 1000) annotation(
    Placement(transformation(origin = {-194, 36}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Blocks.Sources.Ramp ramp(height = 1600, duration(displayUnit = "s") = 100000, offset = 1, startTime = 40000)  annotation(
    Placement(transformation(origin = {182, -36}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.RealExpression TotalGain1(y = gain1.y/1000)  annotation(
    Placement(transformation(origin = {236, -88}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integratorFVolumn1 annotation(
    Placement(transformation(origin = {276, -88}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interaction.Show.RealValue realValueGain1 annotation(
    Placement(transformation(origin = {316, -88}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(battery1.p, converter_ACDC.pin_DC) annotation(
    Line(points = {{20, 70}, {90, 70}, {90, 54}, {100, 54}}, color = {0, 0, 255}));
  connect(electricalGrid.p, converter_ACDC.pin_AC) annotation(
    Line(points = {{146, 52}, {131, 52}, {131, 54}, {116, 54}}, color = {0, 140, 72}));
  connect(transformer.pin_prim, converter_ACDC.pin_DC) annotation(
    Line(points = {{30, 52}, {66, 52}, {66, 54}, {100, 54}}, color = {0, 140, 72}));
  connect(electric_purchasePrice.y, electricalGrid.price) annotation(
    Line(points = {{186, 52}, {166, 52}}, color = {0, 0, 127}));
  connect(TotalGeneratorSOC.y, masterControllerSingleBattery.SOC_Tank) annotation(
    Line(points = {{-60, 153}, {-60, 68}}, color = {0, 0, 127}));
  connect(LoadPower.y, masterControllerSingleBattery.P_load_1) annotation(
    Line(points = {{-119, 62}, {-80, 62}}, color = {0, 0, 127}));
  connect(RenewablePower.y, masterControllerSingleBattery.P_renew_1) annotation(
    Line(points = {{-119, 78}, {-87.5, 78}, {-87.5, 55.5}, {-80, 55.5}}, color = {0, 0, 127}));
  connect(BattSOC.y, masterControllerSingleBattery.SOC_1) annotation(
    Line(points = {{-119, 94}, {-119, 49}, {-80, 49}}, color = {0, 0, 127}));
  connect(masterControllerSingleBattery.P_charge_1, battery1.P_charge) annotation(
    Line(points = {{-41, 61}, {-19, 61}, {-19, 74}, {2, 74}}, color = {0, 0, 127}));
  connect(TankSOC.y, dualCellSplitter.SOC1) annotation(
    Line(points = {{-28, 155}, {-28, 149.5}, {-14, 149.5}, {-14, 142}}, color = {0, 0, 127}));
  connect(TankSOC2.y, dualCellSplitter.SOC2) annotation(
    Line(points = {{-6, 155}, {-6, 142}, {-7, 142}}, color = {0, 0, 127}));
  connect(dualCellSplitter.y, fC250kcurrent2.u) annotation(
    Line(points = {{11, 136}, {18, 136}, {18, 171}, {34, 171}}, color = {0, 0, 127}));
  connect(dualCellSplitter.y2, fC250kcurrent3.u) annotation(
    Line(points = {{11, 132}, {18, 132}, {18, 133}, {34, 133}}, color = {0, 0, 127}));
  connect(fC250kcurrent2.y, fuelCell2.I_control) annotation(
    Line(points = {{61, 168}, {82, 168}, {82, 176}, {99, 176}}, color = {0, 0, 127}));
  connect(tank2.outlet, fuelCell2.fluidPort) annotation(
    Line(points = {{76, 142}, {90, 142}, {90, 170}, {100, 170}}, color = {28, 108, 200}));
  connect(fC250kcurrent3.y, fuelCell3.I_control) annotation(
    Line(points = {{61, 130}, {80.5, 130}, {80.5, 116}, {99, 116}}, color = {0, 0, 127}));
  connect(tank3.outlet, fuelCell3.fluidPort) annotation(
    Line(points = {{78, 86}, {86, 86}, {86, 110}, {100, 110}}, color = {28, 108, 200}));
  connect(fuelCell3.pin, constantVoltage3.p) annotation(
    Line(points = {{120, 110}, {140, 110}}, color = {0, 0, 255}));
  connect(fuelCell2.pin, constantVoltage2.p) annotation(
    Line(points = {{120, 170}, {130, 170}, {130, 160}, {140, 160}}, color = {0, 0, 255}));
  connect(constantVoltage2.n, bus2.term) annotation(
    Line(points = {{160, 160}, {188, 160}, {188, 162}}, color = {0, 0, 255}));
  connect(constantVoltage3.n, bus3.term) annotation(
    Line(points = {{160, 110}, {188, 110}}, color = {0, 0, 255}));
  connect(bus3.term, converter_ACDC.pin_DC) annotation(
    Line(points = {{188, 110}, {192, 110}, {192, 78}, {94, 78}, {94, 54}, {100, 54}}, color = {0, 0, 255}));
  connect(bus2.term, converter_ACDC.pin_DC) annotation(
    Line(points = {{188, 162}, {198, 162}, {198, 88}, {96, 88}, {96, 54}, {100, 54}}, color = {0, 0, 255}));
  connect(gain1.y, electricLoad.P) annotation(
    Line(points = {{113, 0}, {95.5, 0}, {95.5, 29}, {82, 29}}, color = {0, 0, 127}));
  connect(time_expre2.y, combiTable1Ds.u) annotation(
    Line(points = {{259, 0}, {231, 0}, {231, 2}, {206, 2}}, color = {0, 0, 127}));
  connect(TotalGeneratorCO2.y, integratorCO2.u) annotation(
    Line(points = {{141, -88}, {157, -88}}, color = {0, 0, 127}));
  connect(integratorCO2.y, realValueCO2.numberPort) annotation(
    Line(points = {{181, -88}, {197, -88}}, color = {0, 0, 127}));
  connect(TotalFuelMass.y, integratorFMass.u) annotation(
    Line(points = {{141, -110}, {157, -110}}, color = {0, 0, 127}));
  connect(integratorFMass.y, realValueFuelMass.numberPort) annotation(
    Line(points = {{181, -110}, {197, -110}}, color = {0, 0, 127}));
  connect(TotalFuelVolumn.y, integratorFVolumn.u) annotation(
    Line(points = {{141, -130}, {157, -130}}, color = {0, 0, 127}));
  connect(integratorFVolumn.y, realValueFuelVolumn.numberPort) annotation(
    Line(points = {{181, -130}, {197, -130}}, color = {0, 0, 127}));
  connect(InstantFuelConsumption.y, division.u1) annotation(
    Line(points = {{-209, -116}, {-199, -116}}, color = {0, 0, 127}));
  connect(ConstP_Rat.y, division.u2) annotation(
    Line(points = {{-209, -130}, {-199, -130}, {-199, -128}}, color = {0, 0, 127}));
  connect(division.y, product.u2) annotation(
    Line(points = {{-175, -122}, {-161, -122}}, color = {0, 0, 127}));
  connect(product.y, SpecificFuelConsumption.numberPort) annotation(
    Line(points = {{-137, -116}, {-124.5, -116}}, color = {0, 0, 127}));
  connect(ConstSecondPerHour.y, product.u1) annotation(
    Line(points = {{-175, -98}, {-169, -98}, {-169, -110}, {-161, -110}}, color = {0, 0, 127}));
  connect(AlternativeGeneratorCO2.y, integratorAltCO2.u) annotation(
    Line(points = {{142, -170}, {158, -170}}, color = {0, 0, 127}));
  connect(AlternativeFuelMass.y, integratorAltFMass.u) annotation(
    Line(points = {{142, -190}, {158, -190}}, color = {0, 0, 127}));
  connect(AlternativeFuelVolumn.y, integratorAltFVolumn.u) annotation(
    Line(points = {{142, -210}, {158, -210}}, color = {0, 0, 127}));
  connect(integratorAltCO2.y, realValueAltCO2.numberPort) annotation(
    Line(points = {{182, -170}, {198, -170}}, color = {0, 0, 127}));
  connect(integratorAltFMass.y, realValueAltFuelMass.numberPort) annotation(
    Line(points = {{182, -190}, {198, -190}}, color = {0, 0, 127}));
  connect(integratorAltFVolumn.y, realValueAltFVolumn.numberPort) annotation(
    Line(points = {{182, -210}, {198, -210}}, color = {0, 0, 127}));
  connect(transformer.pin_sec, electricLoad.n) annotation(
    Line(points = {{30, 40}, {34, 40}, {34, 30}, {60, 30}}, color = {0, 140, 72}));
  connect(dualCellSplitter.u, masterControllerSingleBattery.ControllerFC_OutPut) annotation(
    Line(points = {{-18, 134}, {-50, 134}, {-50, 66}}, color = {0, 0, 127}));
  connect(masterControllerSingleBattery.P_discharge_1, battery1.P_discharge) annotation(
    Line(points = {{-40, 56}, {-14, 56}, {-14, 66}, {2, 66}}, color = {0, 0, 127}));
  connect(masterControllerSingleBattery.P_dispatch[1], generator1.P_sp) annotation(
    Line(points = {{-40, 40}, {-10, 40}, {-10, 20}}, color = {0, 0, 127}));
  connect(masterControllerSingleBattery.P_dispatch[2], generator2.P_sp) annotation(
    Line(points = {{-40, 40}, {-30, 40}, {-30, -8}, {-10, -8}, {-10, -16}}, color = {0, 0, 127}));
  connect(masterControllerSingleBattery.P_dispatch[3], generator3.P_sp) annotation(
    Line(points = {{-40, 40}, {-30, 40}, {-30, -40}, {-10, -40}, {-10, -46}}, color = {0, 0, 127}));
  connect(masterControllerSingleBattery.P_alt_dispatch[1], generator_af_1.P_sp) annotation(
    Line(points = {{-40, 34}, {-30, 34}, {-30, -72}, {-10, -72}, {-10, -80}}, color = {0, 0, 127}));
  connect(masterControllerSingleBattery.P_alt_dispatch[2], generator_af_2.P_sp) annotation(
    Line(points = {{-40, 34}, {-30, 34}, {-30, -110}, {-10, -110}, {-10, -120}}, color = {0, 0, 127}));
  connect(masterControllerSingleBattery.P_alt_dispatch[3], generator_af_3.P_sp) annotation(
    Line(points = {{-40, 34}, {-30, 34}, {-30, -152}, {-10, -152}, {-10, -160}}, color = {0, 0, 127}));
  connect(idealTankMultiPort.fluidPort2, generator1.outlet_fuel) annotation(
    Line(points = {{60, -25}, {34, -25}, {34, -4}, {-6, -4}, {-6, 0}}, color = {28, 108, 200}));
  connect(idealTankMultiPort.fluidPort, generator2.outlet_fuel) annotation(
    Line(points = {{60, -28}, {26, -28}, {26, -38}, {-6, -38}, {-6, -34}}, color = {28, 108, 200}));
  connect(idealTankMultiPort.fluidPort3, generator3.outlet_fuel) annotation(
    Line(points = {{60, -30}, {44, -30}, {44, -74}, {-6, -74}, {-6, -66}}, color = {28, 108, 200}));
  connect(alt_fuel_tank.fluidPort2, generator_af_1.outlet_fuel) annotation(
    Line(points = {{60, -66}, {50, -66}, {50, -108}, {-6, -108}, {-6, -100}}, color = {28, 108, 200}));
  connect(alt_fuel_tank.fluidPort, generator_af_2.outlet_fuel) annotation(
    Line(points = {{60, -70}, {52, -70}, {52, -150}, {-6, -150}, {-6, -140}}, color = {28, 108, 200}));
  connect(alt_fuel_tank.fluidPort3, generator_af_3.outlet_fuel) annotation(
    Line(points = {{60, -72}, {56, -72}, {56, -198}, {-4, -198}, {-4, -190}, {-6, -190}, {-6, -180}}, color = {28, 108, 200}));
  connect(generator1.pin_AC, transformer.pin_sec) annotation(
    Line(points = {{0, 10}, {30, 10}, {30, 40}}, color = {0, 140, 72}));
  connect(generator2.pin_AC, transformer.pin_sec) annotation(
    Line(points = {{0, -24}, {30, -24}, {30, 40}}, color = {0, 140, 72}));
  connect(generator3.pin_AC, transformer.pin_sec) annotation(
    Line(points = {{0, -56}, {30, -56}, {30, 40}}, color = {0, 140, 72}));
  connect(generator_af_1.pin_AC, transformer.pin_sec) annotation(
    Line(points = {{0, -90}, {30, -90}, {30, 40}}, color = {0, 140, 72}));
  connect(generator_af_2.pin_AC, transformer.pin_sec) annotation(
    Line(points = {{0, -130}, {30, -130}, {30, 40}}, color = {0, 140, 72}));
  connect(generator_af_3.pin_AC, transformer.pin_sec) annotation(
    Line(points = {{0, -170}, {30, -170}, {30, 40}}, color = {0, 140, 72}));
  connect(dieselTankSOC_1.y, masterControllerSingleBattery.TankmultipleSOC[1]) annotation(
    Line(points = {{-170, 1}, {-170, 20}, {-56, 20}, {-56, 34}}, color = {0, 0, 127}));
  connect(dieselTankSOC_2.y, masterControllerSingleBattery.TankmultipleSOC[2]) annotation(
    Line(points = {{-150, 1}, {-150, 16}, {-56, 16}, {-56, 34}}, color = {0, 0, 127}));
  connect(dieselTankSOC_3.y, masterControllerSingleBattery.TankmultipleSOC[3]) annotation(
    Line(points = {{-130, 1}, {-130, 12}, {-56, 12}, {-56, 34}}, color = {0, 0, 127}));
  connect(altFuelTankSOC_1.y, masterControllerSingleBattery.AltFuelTankSOC[1]) annotation(
    Line(points = {{-110, -21}, {-110, -26}, {-46, -26}, {-46, 34}}, color = {0, 0, 127}));
  connect(altFuelTankSOC_2.y, masterControllerSingleBattery.AltFuelTankSOC[2]) annotation(
    Line(points = {{-90, -21}, {-90, -26}, {-46, -26}, {-46, 34}}, color = {0, 0, 127}));
  connect(altFuelTankSOC_3.y, masterControllerSingleBattery.AltFuelTankSOC[3]) annotation(
    Line(points = {{-70, -21}, {-70, -25.5}, {-46, -25.5}, {-46, 34}}, color = {0, 0, 127}));
  connect(gainAltUpperBound.y, masterControllerSingleBattery.altFuelUpperBound) annotation(
    Line(points = {{-190, 144}, {-162, 144}, {-162, 44}, {-80, 44}}, color = {0, 0, 127}));
  connect(gainAltLowerBound.y, masterControllerSingleBattery.altFuelLowerBound) annotation(
    Line(points = {{-190, 132}, {-170, 132}, {-170, 42}, {-80, 42}}, color = {0, 0, 127}));
  connect(gainAltMinBSFC.y, masterControllerSingleBattery.altFuelMinBSFC) annotation(
    Line(points = {{-190, 118}, {-148, 118}, {-148, 38}, {-80, 38}}, color = {0, 0, 127}));
  connect(gainDieselUpperBound.y, masterControllerSingleBattery.dieselUpperBound) annotation(
    Line(points = {{-190, 70}, {-178, 70}, {-178, 36}, {-80, 36}}, color = {0, 0, 127}));
  connect(gainDieselLowerBound.y, masterControllerSingleBattery.dieselLowerBound) annotation(
    Line(points = {{-190, 52}, {-182, 52}, {-182, 34}, {-80, 34}}, color = {0, 0, 127}));
  connect(gainDieselMinBSFC.y, masterControllerSingleBattery.dieselMinimalBSFC) annotation(
    Line(points = {{-190, 36}, {-182, 36}, {-182, 32}, {-80, 32}}, color = {0, 0, 127}));
  connect(altFuel_bsfc_table.pwr_higher_band, gainAltUpperBound.u) annotation(
    Line(points = {{-219, 139}, {-210, 139}, {-210, 144}, {-198, 144}}, color = {0, 0, 127}));
  connect(altFuel_bsfc_table.pwr_lower_band, gainAltLowerBound.u) annotation(
    Line(points = {{-219, 132}, {-198, 132}}, color = {0, 0, 127}));
  connect(altFuel_bsfc_table.bsfc_min_out, gainAltMinBSFC.u) annotation(
    Line(points = {{-219, 125}, {-210, 125}, {-210, 118}, {-199, 118}}, color = {0, 0, 127}));
  connect(diesel_bsfc_table.pwr_higher_band, gainDieselUpperBound.u) annotation(
    Line(points = {{-221, 59}, {-210, 59}, {-210, 70}, {-198, 70}}, color = {0, 0, 127}));
  connect(diesel_bsfc_table.pwr_lower_band, gainDieselLowerBound.u) annotation(
    Line(points = {{-221, 52}, {-198, 52}}, color = {0, 0, 127}));
  connect(diesel_bsfc_table.bsfc_min_out, gainDieselMinBSFC.u) annotation(
    Line(points = {{-221, 45}, {-210, 45}, {-210, 36}, {-198, 36}}, color = {0, 0, 127}));
  connect(combiTable1Ds.y[1], gain1.u) annotation(
    Line(points = {{183, 2}, {137, 2}, {137, 0}, {136, 0}}, color = {0, 0, 127}));
  connect(TotalGain1.y, integratorFVolumn1.u) annotation(
    Line(points = {{247, -88}, {263, -88}}, color = {0, 0, 127}));
  connect(integratorFVolumn1.y, realValueGain1.numberPort) annotation(
    Line(points = {{287, -88}, {303, -88}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Rectangle(origin = {40, -40}, fillColor = {0, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-40, 40}, {40, -40}}), Rectangle(origin = {40, 40}, fillColor = {85, 85, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-40, 40}, {40, -40}}), Rectangle(origin = {-40, -40}, fillColor = {85, 85, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-40, 40}, {40, -40}}), Rectangle(origin = {40, -40}, fillColor = {0, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-40, 40}, {40, -40}}), Rectangle(origin = {-40, 40}, fillColor = {0, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-40, 40}, {40, -40}})}),
    uses(Modelica(version = "4.0.0")),
    Diagram(coordinateSystem(extent = {{-240, 180}, {220, -220}})),
    version = "",
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end AZEAT_FortunaCrane_Short_ControllerGroup;
