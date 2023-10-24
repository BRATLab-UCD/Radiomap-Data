#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream> 

#include "network_planning_5g_tdd.h"

#define API_DATA_FOLDER "C:/Users/Administrator/PycharmProjects/Data/"

#define SERVICES_5G 1
#define MAX_CELLS 9

int main(int argc, char** argv)
{
	for (int i = 1; i <= 100; ++i) {
		int Error = 0;
		int ProjectHandle = 0;
		int ParaValueInt = 0;
		double ParaValueDouble = 0.;

		// ------ General and Propagation and Network Planing Parameters ------ //
		WinProp_ParaMain GeneralParameters;
		WinProp_Pattern  WinPropPattern[MAX_CELLS];
		WinProp_Antenna  Antenna[MAX_CELLS];
		WinProp_Carrier  Carrier[MAX_CELLS];

		// Propagation results 
		WinProp_Result   PropResults[MAX_CELLS];

		// Prediction heights 
		int              NrPredictionHeights = 1;
		double           PredictionHeights[1] = { 30.0 };

		// ---------------------- Initialisation of parameters ---------------- //
		WinProp_Structure_Init_ParameterMain(&GeneralParameters);

		for (int cell = 0; cell < MAX_CELLS; cell++)
		{
			WinProp_Structure_Init_Pattern(&WinPropPattern[cell]);
			WinProp_Structure_Init_Antenna(&Antenna[cell]);
			WinProp_Structure_Init_Carrier(&Carrier[cell]);
			WinProp_Structure_Init_Result(&PropResults[cell]);
		}

		// ---------------- Definition of parameters -------------------------- //
		// Definition of scenario. 
		GeneralParameters.BuildingsMode = BUILDINGSMODE_BINARY; // load a .odb database 
		std::string my_odb_database_cstr = std::string(API_DATA_FOLDER) + "high_resolution_urbanmap_odbdata/map" + std::to_string(i);// name of .odb database without extensions 
		const char* my_odb_database = my_odb_database_cstr.c_str();
		sprintf(GeneralParameters.BuildingsName, "%s", my_odb_database);
		GeneralParameters.ScenarioMode = SCENARIOMODE_URBAN; // Urban prediction
		GeneralParameters.PredictionModelUrban = PREDMODEL_UDP; // Use IRT Model
		GeneralParameters.BuildingsMaterialsInd = 1;            // Individual material properties for buildings
		GeneralParameters.VegetationMode = VEGETATION_BUILDING; // vegetation from building database
		GeneralParameters.VegetationPropertiesInd = 1;          // Individual material properties for vegetation

		// Size of matrix with results.
		GeneralParameters.Resolution = 2;                        // Resolution in meter
		GeneralParameters.NrLayers = NrPredictionHeights;          // Number of prediction heights
		GeneralParameters.PredictionHeights = PredictionHeights;   // Prediction height in meter

		//// Get prediction area.

		int NumberCorners;
		int DatabaseFileType = DATABASE_TYPE_WINPROP_ODB;
		struct COORDPOINT* Polygon;
		double* Resolution = &GeneralParameters.Resolution;
		int NumberHeights = 0;
		int PolygonUsed = 0;

		OutdoorPlugIn_ReadPredictionArea(my_odb_database, &NumberCorners, &Polygon, DatabaseFileType, Resolution, &NumberHeights, nullptr, &PolygonUsed);

		//for (int i = 0; i < NumberCorners; ++i) {
		//	std::cout << "Polygon[" << i << "].x: " << Polygon[i].x << std::endl;
		//	std::cout << "Polygon[" << i << "].y: " << Polygon[i].y << std::endl;
		//	std::cout << "Polygon[" << i << "].z: " << Polygon[i].z << std::endl;
		//}

		//return 0;

		//// Definition of prediction area.

		GeneralParameters.UrbanLowerLeftX = Polygon[0].x;
		GeneralParameters.UrbanLowerLeftY = Polygon[0].y;
		GeneralParameters.UrbanUpperRightX = Polygon[2].x;
		GeneralParameters.UrbanUpperRightY = Polygon[2].y;

		// Copy coordinates to prediction area of second model (not yet used).
		double LowerLeftX = GeneralParameters.UrbanLowerLeftX;
		double LowerLeftY = GeneralParameters.UrbanLowerLeftY;
		double UpperRightX = GeneralParameters.UrbanUpperRightX;
		double UpperRightY = GeneralParameters.UrbanUpperRightY;

		// Break point parameters
		GeneralParameters.BreakpointMode = BREAKPOINT_DEFAULT;
		GeneralParameters.BreakpointFactor = 4.;
		GeneralParameters.BreakpointOffset = 0;
		GeneralParameters.Resolution = 2;                        // Resolution in meter
		// Selection of paths
		GeneralParameters.MaxPathLossEnabled = 1;
		GeneralParameters.MaxPathLoss = 200.f;
		GeneralParameters.MaxDynamicPathsEnabled = 1;
		GeneralParameters.MaxDynamicPaths = 100.f;
		GeneralParameters.MaxNumberPathsUsed = 1;
		GeneralParameters.MaxNumberPaths = 20;

		// Antenna patten
		WinPropPattern[0].Mode = PATTERN_MODE_FILE;   // Load pattern from file
		char PatternFileName[500];
		sprintf(PatternFileName, "%s", API_DATA_FOLDER "antennas/18dBi.msi");
		WinPropPattern[0].Filename = PatternFileName; // Pattern file

		// Desired propagation results in WinProp result format
		WinProp_Propagation_Results OutputResults;
		WinProp_Structure_Init_Propagation_Results(&OutputResults);
		char ResultPath[500];
		sprintf(ResultPath, "%s", API_DATA_FOLDER "output/propagation");
		OutputResults.ResultPath = ResultPath; // Output data directory 
		OutputResults.Delay = 0;
		OutputResults.StatusLOS = 0;
		OutputResults.AdditionalResultsASCII = 1;
		GeneralParameters.OutputResults = &OutputResults;

		// Callback functions.
		WinProp_Callback    Callback;
		Callback.Percentage = CallbackProgress;
		Callback.Message = CallbackMessage;
		Callback.Error = CallbackError;

		//--------------------------------------------------------------------------
		// Compute wave propagation for cells
		//--------------------------------------------------------------------------

		//// ----------------  Define IRT parameters -------------------------- //
		//Model_UrbanIRT ParameterIRT;
		//WinProp_Structure_Init_Model_IRT(&ParameterIRT);

		//// set path loss exponents 
		//ParameterIRT.BreakpointExponentBeforeLOS = 2.3;
		//ParameterIRT.BreakpointExponentAfterLOS = 3.3;
		//ParameterIRT.BreakpointExponentBefore = 2.5;
		//ParameterIRT.BreakpointExponentAfter = 3.3;

		//// Number of interactions
		//ParameterIRT.MaxReflections = 2;
		//ParameterIRT.MaxDiffractions = 1;
		//ParameterIRT.MaxScatterings = 0;
		//ParameterIRT.MaxSumReflDiff = 2;

		//// Superposition and diffraction mode
		//ParameterIRT.Superposition = 0;
		//ParameterIRT.DiffractionModel = 'e';
		//ParameterIRT.PostProcessingMode = 0;
		//ParameterIRT.Postprocessing = 0;
		//ParameterIRT.FreeSpaceCancel = 0;

		// --------------------------  antennas  -------------------------------- //
		// Position and configuration of the antennas (cells).
		double Position1x = LowerLeftX + 1.0 / 4 * (UpperRightX - LowerLeftX);
		double Position1y = LowerLeftY + 3.0 / 4 * (UpperRightY - LowerLeftY);
		double Position2x = LowerLeftX + 3.0 / 4 * (UpperRightX - LowerLeftX);
		double Position2y = LowerLeftY + 3.0 / 4 * (UpperRightY - LowerLeftY);
		double Position3x = LowerLeftX + 1.0 / 2 * (UpperRightX - LowerLeftX);
		double Position3y = LowerLeftY + 1.0 / 4 * (UpperRightY - LowerLeftY);

		char AntennaName[MAX_CELLS][500] = { "Cell_1_A1", "Cell_1_A2", "Cell_1_A3", "Cell_2_A1", "Cell_2_A2", "Cell_2_A3", "Cell_3_A1", "Cell_3_A2" , "Cell_3_A3" };
		double SiteX[MAX_CELLS] = { Position1x, Position1x, Position1x, Position2x, Position2x, Position2x, Position3x, Position3x, Position3x };
		double SiteY[MAX_CELLS] = { Position1y, Position1y, Position1y, Position2y, Position2y, Position2y, Position3y, Position3y, Position3y };
		double SiteZ[MAX_CELLS] = { 15., 15., 15., 15., 15., 15., 15., 15., 15. };
		double Power[MAX_CELLS] = { 46., 46., 46., 46., 46., 46., 46., 46., 46. }; // Power in dBm
		double Azmith[MAX_CELLS] = { 0., 120., 240., 0., 120., 240., 0., 120., 240. };
		double Downtilt[MAX_CELLS] = { 10., 10., 10., 10., 10., 10., 10., 10., 10. };
		double Frequency[MAX_CELLS] = { 4750.0 }; // same frequencies in MHz as will defined for the carriers (for network planning)
		int AntennaCarrier[MAX_CELLS] = { 1 };

		for (int cell = 0; cell < MAX_CELLS; cell++)
		{
			// set antenna pattern
			Antenna[cell].Pattern = &WinPropPattern[0];

			// Set antenna properties now.
			AntennaPropertiesSet(&Antenna[cell], SiteX[cell], SiteY[cell], SiteZ[cell], Frequency[0], AntennaName[cell],
				Power[cell], WINPROP_POWER_OUTPUT, WINPROP_MODEL_DPM, Azmith[cell], Downtilt[cell], cell + 1);

			Carrier[cell].CarrierID = AntennaCarrier[0];
			Carrier[cell].NrAntennaBeams = 1;
			Carrier[cell].SymbolsPerSlot = 14;
			Carrier[cell].Numerology = 1;
			Carrier[cell].TDD_Slots_DL = 7;
			Carrier[cell].TDD_Slots_UL = 7;
			Carrier[cell].TDD_Slots_Flex = 0;
			Carrier[cell].BeamGainCtrlServer = 0.;
			Carrier[cell].BeamGainCtrlInterferer = 0.;
			Carrier[cell].BeamGainDataServer = 0.;
			Carrier[cell].BeamGainDataInterferer = 0.;

			CarrierPropertiesSet(&Antenna[cell], &Carrier[cell]);

			/*--------------------- Compute outdoor prediction ------------------------*/
			Error = OutdoorPlugIn_ComputePrediction(&Antenna[cell], &GeneralParameters, NULL, NULL, NULL, NULL, &Callback, &PropResults[cell], NULL, NULL, NULL);

			/*---------------------- write propagation results ------------------------*/
			if (Error == 0)
			{
				char NameForOutput[200];
				sprintf(NameForOutput, "%s%s%s%s", API_DATA_FOLDER, "output/propagation/", Antenna[cell].Name, ".txt");

				write_ascii(&PropResults[cell], NameForOutput);
			}
			else
			{
				/* Error during prediction. Print error message. */
				CallbackError(GeneralParameters.ErrorMessageMain, Error);
			}
		}


		// -------------------- Network Planning Project ----------------------- /

		// Create a new network planning project based on 5G TDD air interface                           
		if (Error == 0)
		{
			Error = WinProp_Net_Project_Open(&ProjectHandle, NET_AIRINTERFACE_5G_TDD_GENERIC, NULL);
		}

		// Add all propagation maps which have been computed.
		if (Error == 0)
		{
			int MapIndex[MAX_CELLS];
			for (int Count = 0; Count < MAX_CELLS; Count++)
			{
				if (Error == 0)
				{
					Error = WinProp_Net_PropagationMap_Add(ProjectHandle, &MapIndex[Count], &Antenna[Count], &PropResults[Count]);
				}
			}
		}

		// Set a name for the project.
		if (Error == 0)
		{
			char ProjectName[200];
			sprintf(ProjectName, "%s", "TDD-5G");
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_PROJECTNAME, NULL, NULL, ProjectName);
		}

		// -------------------  numerology  ----------------------- //
		// Set numerology and bandwidth
		if (Error == 0)
		{
			ParaValueInt = NET_PARA_5G_NUMEROLOGY1_100MHZ_FR1;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_BANDWIDTH_MODE_5G, NULL, &ParaValueInt, NULL);
		}

		// -------------------  TDD  ----------------------- //
		if (Error == 0)
		{
			ParaValueInt = 1; // TDD mode
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_DUPLEX_SUBMODE, NULL, &ParaValueInt, NULL);

		}

		if (Error == 0)
		{
			ParaValueInt = 3; // TDD properties specified for each carrier individually
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_TDD_VARIABLE_SWITCH, NULL, &ParaValueInt, NULL);
		}

		// --------------------------  Carriers  -------------------------------- //
		// Add carriers in n1 band.
		if (Error == 0)
		{
			double FrequencyUL = 0.;
			double FrequencyDL = 0.;
			double CarrierSeparation = 100.;
			for (int CurrentCarrier = 0; CurrentCarrier < 1; CurrentCarrier++)
			{
				FrequencyUL = Frequency[0] + (CurrentCarrier * CarrierSeparation);
				FrequencyDL = Frequency[0] + (CurrentCarrier * CarrierSeparation);
				Error = WinProp_Net_Carrier_Add(ProjectHandle, CurrentCarrier + 1);

				Error = WinProp_Net_Carrier_Para_Set(
					ProjectHandle, CurrentCarrier + 1, NET_PARA_CARRIER_FREQ_DL, &FrequencyDL, NULL, NULL);
				Error = WinProp_Net_Carrier_Para_Set(
					ProjectHandle, CurrentCarrier + 1, NET_PARA_CARRIER_FREQ_UL, &FrequencyUL, NULL, NULL);
			}
		}

		// Check if number of carriers is correct. 
		if (Error == 0)
		{
			int NrCarriers = 0;
			Error = WinProp_Net_Project_Para_Get(ProjectHandle, NET_PARA_CARRIERS, NULL, &NrCarriers, NULL);
			if (Error == 0)
			{
				if (NrCarriers != 1)
					Error = 1;
			}
		}

		// -------------------------  Transmission modes  --------------------------- //
		// Define 5G transmission modes
		if (Error == 0)
		{
			//const char* TransmissionMode_Name[SERVICES_5G] = {
			//									 "00 - QPSK - R 120",
			//									 "01 - QPSK - R 193",
			//									 "02 - QPSK - R 308",
			//									 "03 - QPSK - R 449",
			//									 "04 - QPSK - R 602",
			//									 "06 - 16 QAM - R 378",
			//									 "07 - 16 QAM - R 490",
			//									 "08 - 16 QAM - R 658",
			//									 "09 - 64 QAM - R 466",
			//									 "10 - 64 QAM - R 666",
			//									 "11 - 64 QAM - R 873",
			//									 "12 - 256 QAM - R 797",
			//									 "13 - 256 QAM - R 948" };
			//double              TransmissionMode_Bitrate[SERVICES_5G] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			//double              TransmissionMode_SNIR[SERVICES_5G] = { -3.0, -2.0, -1.0, 0.0, 2.0, 7.0, 10.0, 12.0, 13.0, 16.0, 18.0, 22.0,  27.0 };
			//int                 TransmissionMode_Coderate_K[SERVICES_5G] = { 120, 193, 308, 449, 602, 378, 490, 658, 466, 666, 873, 797, 948 };
			//int                 TransmissionMode_Coderate_N[SERVICES_5G] = { 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024 };
			//int                 TransmissionMode_MCS[SERVICES_5G] = { 2, 2, 2, 2, 2, 4, 4, 4, 6, 6, 6, 8, 8 };
			//double              TransmissionMode_Backoff[SERVICES_5G] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 3.0, 3.0, 3.0, 3.0 };
			//int                 TransmissionMode_Resource_Blocks_DL[SERVICES_5G] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
			//int                 TransmissionMode_Resource_Blocks_UL[SERVICES_5G] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,1, 1, 1 };
			//double              TransmissionMode_Overhead_Ratio_DL[SERVICES_5G] = { 14., 14., 14., 14., 14., 14., 14., 14., 14., 14., 14., 14. , 14. };
			//double              TransmissionMode_Overhead_Ratio_UL[SERVICES_5G] = { 8., 8., 8., 8., 8., 8., 8., 8., 8., 8., 8., 8., 8. };

			const char* TransmissionMode_Name[SERVICES_5G] = { "07 - 16 QAM - R 490" };
			double              TransmissionMode_Bitrate[SERVICES_5G] = { 0 };
			double              TransmissionMode_SNIR[SERVICES_5G] = { 10.0 };
			int                 TransmissionMode_Coderate_K[SERVICES_5G] = { 490 };
			int                 TransmissionMode_Coderate_N[SERVICES_5G] = { 1024 };
			int                 TransmissionMode_MCS[SERVICES_5G] = { 4 };
			double              TransmissionMode_Backoff[SERVICES_5G] = { 0.0 };
			int                 TransmissionMode_Resource_Blocks_DL[SERVICES_5G] = { 1 };
			int                 TransmissionMode_Resource_Blocks_UL[SERVICES_5G] = { 1 };
			double              TransmissionMode_Overhead_Ratio_DL[SERVICES_5G] = { 14. };
			double              TransmissionMode_Overhead_Ratio_UL[SERVICES_5G] = { 8. };

			for (int CurrentService = 0; CurrentService < SERVICES_5G; CurrentService++)
			{
				// Add new service. 
				char ServiceName[100];
				sprintf(ServiceName, "%s", TransmissionMode_Name[CurrentService]);
				Error = WinProp_Net_TransmissionMode_Add(ProjectHandle, ServiceName, CurrentService);

				// Set position/priority.
				ParaValueInt = SERVICES_5G - CurrentService;
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_POSITION, NULL, &ParaValueInt, NULL);

				// Set bitrate.
				ParaValueDouble = TransmissionMode_Bitrate[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_BITRATE_DL, &ParaValueDouble, NULL, NULL);
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_BITRATE_UL, &ParaValueDouble, NULL, NULL);

				// Set code rate.
				ParaValueInt = TransmissionMode_Coderate_K[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_CODERATE_K_DL, NULL, &ParaValueInt, NULL);
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_CODERATE_K_UL, NULL, &ParaValueInt, NULL);

				ParaValueInt = TransmissionMode_Coderate_N[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_CODERATE_N_DL, NULL, &ParaValueInt, NULL);
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_CODERATE_N_UL, NULL, &ParaValueInt, NULL);

				// Set number of resource blocks.
				ParaValueInt = TransmissionMode_Resource_Blocks_DL[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_RESOURCE_BLOCKS_DL, NULL, &ParaValueInt, NULL);
				ParaValueInt = TransmissionMode_Resource_Blocks_UL[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_RESOURCE_BLOCKS_UL, NULL, &ParaValueInt, NULL);

				// Set overhead ratio. 
				ParaValueDouble = TransmissionMode_Overhead_Ratio_UL[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_OVERHEAD_RATIO_UL, &ParaValueDouble, NULL, NULL);
				ParaValueDouble = TransmissionMode_Overhead_Ratio_DL[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_OVERHEAD_RATIO_DL, &ParaValueDouble, NULL, NULL);

				// Set required SNIR.
				ParaValueDouble = TransmissionMode_SNIR[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_SNIR_DL, &ParaValueDouble, NULL, NULL);
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_SNIR_UL, &ParaValueDouble, NULL, NULL);

				// Set modulation scheme.
				ParaValueInt = TransmissionMode_MCS[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_MODULATION_UL, NULL, &ParaValueInt, NULL);
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_MODULATION_DL, NULL, &ParaValueInt, NULL);

				// Set power backoff. 
				ParaValueDouble = TransmissionMode_Backoff[CurrentService];
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_POWER_BACKOFF_UL, &ParaValueDouble, NULL, NULL);
				Error = WinProp_Net_TransmissionMode_Para_Set(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_POWER_BACKOFF_DL, &ParaValueDouble, NULL, NULL);

				// Get bitrate in kBit/s for current transmission mode. Bitrate is automatically computed based
				// on parameters previously set and general air interface parameters.
				Error = WinProp_Net_TransmissionMode_Para_Get(
					ProjectHandle, CurrentService, NET_PARA_TRANS_MODE_BITRATE_DL, &ParaValueDouble, NULL, NULL);
				TransmissionMode_Bitrate[CurrentService] = ParaValueDouble;
			}
		}

		// Check if number of services is correct.
		if (Error == 0)
		{
			int NrServices = 0;
			Error = WinProp_Net_Project_Para_Get(ProjectHandle, NET_PARA_SERVICES, NULL, &NrServices, NULL);
			if (Error == 0)
			{
				if (NrServices != SERVICES_5G)
					Error = 1;
			}
		}

		// --------------------------  Project parameters  -------------------------- //

		// Set resolution for result matrix.
		if (Error == 0)
		{
			ParaValueDouble = GeneralParameters.Resolution;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_RESOLUTION, &ParaValueDouble, NULL, NULL);
		}
		// Set the simulation area
		if (Error == 0)
		{
			ParaValueDouble = GeneralParameters.UrbanLowerLeftX;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_AREA_LL_X, &ParaValueDouble, NULL, NULL);
		}
		if (Error == 0)
		{
			ParaValueDouble = GeneralParameters.UrbanLowerLeftY;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_AREA_LL_Y, &ParaValueDouble, NULL, NULL);
		}
		if (Error == 0)
		{
			ParaValueDouble = GeneralParameters.UrbanUpperRightX;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_AREA_UR_X, &ParaValueDouble, NULL, NULL);
		}
		if (Error == 0)
		{
			ParaValueDouble = GeneralParameters.UrbanUpperRightY;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_AREA_UR_Y, &ParaValueDouble, NULL, NULL);
		}

		// Set paths for additional output in WinProp file format.
		if (Error == 0)
		{
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_OUTPUT_WINPROP, NULL, NULL, API_DATA_FOLDER "output/network");
		}

		// ------------------------  Cell assignment   ------------------------------ //
		if (Error == 0)
		{
			int IntValue = 0; // highest Rx power for all carries mode
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_CELL_ASSIGNMENT_MODE, NULL, &IntValue, NULL);
		}

		// Set Min required SNIR
		if (Error == 0)
		{
			ParaValueDouble = -5.4;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_CELL_ASSIGNMENT_MIN_REQ_SNIR, &ParaValueDouble, NULL, NULL);
		}

		// Set Min required power
		if (Error == 0)
		{
			ParaValueDouble = -95.;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_CELL_ASSIGNMENT_MIN_REQ_POWER, &ParaValueDouble, NULL, NULL);
		}


		// -----------------------  OFDM parameters  -------------------------------- //
		// Set control sub-carriers 
		if (Error == 0)
		{
			ParaValueInt = 2500;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_SUBCARRIERS_CONTROL, NULL, &ParaValueInt, NULL);
		}

		// Set reference sub-carriers 
		if (Error == 0)
		{
			ParaValueInt = 500;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_SUBCARRIERS_REFERENCE, NULL, &ParaValueInt, NULL);
		}

		// Set Guard sub-carriers
		if (Error == 0)
		{
			ParaValueInt = 29;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_SUBCARRIERS_GUARD, NULL, &ParaValueInt, NULL);
		}

		// Set data symbols 
		if (Error == 0)
		{
			ParaValueInt = 10;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_SYMBOLS_DATA, NULL, &ParaValueInt, NULL);
		}

		// Set control symbols 
		if (Error == 0)
		{
			ParaValueInt = 1;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_SYMBOLS_CONTROL, NULL, &ParaValueInt, NULL);
		}

		// Set reference symbols
		if (Error == 0)
		{
			ParaValueInt = 3;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_SYMBOLS_REFERENCE, NULL, &ParaValueInt, NULL);
		}

		// Set FFT order 
		if (Error == 0)
		{
			ParaValueInt = 4096;
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_FFT_SIZE, NULL, &ParaValueInt, NULL);
		}

		// Desired network planning results in WinProp result format
		//if (Error == 0)
		//{
		//	Error = WinProp_Net_Project_Result_Switch(ProjectHandle, NET_RESULT_REC_POWER, 1);
		//	Error = WinProp_Net_Project_Result_Switch(ProjectHandle, NET_RESULT_OFDM_POWER, 1);
		//	Error = WinProp_Net_Project_Result_Switch(ProjectHandle, NET_RESULT_MAX_THROUGHPUT, 1);
		//	Error = WinProp_Net_Project_Result_Switch(ProjectHandle, NET_RESULT_SNIR, 1);
		//}

		//--------------------------------------------------------------------------
		//  Compute network planning results
		//--------------------------------------------------------------------------
		if (Error == 0)
		{
			char HeightString[200];
			sprintf(HeightString, "%s", "");

			// Generate string with height values, e.g. a string like "1.5 2.5 3.5". 
			for (int Height = 0; Height < NrPredictionHeights; Height++)
			{
				// Add current height to string.
				sprintf(HeightString, "%s%.2f ", HeightString, PredictionHeights[Height]);
			}

			// Send heights to WinProp API.
			Error = WinProp_Net_Project_Para_Set(ProjectHandle, NET_PARA_HEIGHT_MULTIPLE, NULL, NULL, HeightString);

			// Start network computation.
			if (Error == 0)
			{
				Error = WinProp_Net_Project_Compute(ProjectHandle, &Callback);
			}
		}

		// ------------------------------------------------------------------------
		// Retrieve  results 
		// ------------------------------------------------------------------------
		// As an example: retrieve max. throughput (kbps) per pixel.
		if (Error == 0)
		{
			WinProp_Result* TOTAL_REC = NULL;
			Error = WinProp_Net_NetworkMap_Get(ProjectHandle, -1, NET_RESULT_TOTAL_REC, &TOTAL_REC);

			// Write max. throughput result to ASCII file.
			if (Error == 0)
			{
				char NameForOutput[200];
				sprintf(NameForOutput, "%s%s%d.txt", API_DATA_FOLDER, "high_resolution_receivedpower_4750MHz/TOTAL_REC_4750MHz_", i);
				write_ascii(TOTAL_REC, NameForOutput);
			}

		}


		// ------------------------------------------------------------------------
		// Free memory
		// ------------------------------------------------------------------------
		for (int Count = 0; Count < MAX_CELLS; Count++)
		{
			// Free propagation results.
			WinProp_FreeResult(&PropResults[Count]);
		}

		// Close network project.
		Error = WinProp_Net_Project_Close(ProjectHandle);
		//return 0;
	}
}

int _STD_CALL CallbackMessage(const char* Text)
{
	if (Text == nullptr)
		return 0;

	std::cout << "\n" << Text;

	return(0);
}

int _STD_CALL CallbackError(const char* Text, int Error)
{
	if (Text == nullptr)
		return 0;

	std::cout << "\n";

#ifdef __LINUX
	std::cout << "\033[31m" << "Error (" << Error << "): "; // highlight error in red color
#else
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hConsole, FOREGROUND_RED);
	std::cout << "Error (" << Error << "): ";
#endif // __LINUX
	std::cout << Text;

#ifdef __LINUX
	std::cout << "\033[0m"; // highlight error in red color
#else
	SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_GREEN);
#endif // __LINUX

	return 0;
}

int _STD_CALL CallbackProgress(int value, const char* text)
{
	char Line[200];

	sprintf(Line, "\n%d%% %s", value, text);
	std::cout << Line;

	return(0);
}

// Helper functions
void write_ascii(const WinProp_Result* Resultmatrix, const char* Filename) {

	if (Resultmatrix)
	{
		FILE* OutputFile = fopen(Filename, "w");
		if (OutputFile)
		{
			/* Loop through WinPropall pixels. */
			for (int x = 0; x < Resultmatrix->Columns; x++)
			{
				for (int y = 0; y < Resultmatrix->Lines; y++)
				{
					/* Compute real coordinates. */
					double Coordinate_X = Resultmatrix->LowerLeftX + ((double)x + 0.5) * Resultmatrix->Resolution;
					double Coordinate_Y = Resultmatrix->LowerLeftY + ((double)y + 0.5) * Resultmatrix->Resolution;

					/* Check if pixel was computed or not */
					if (Resultmatrix->Matrix[0][x][y] > -1000)
					{
						fprintf(OutputFile, "%.5f\t%.5f\t%.4f\n", Coordinate_X, Coordinate_Y, Resultmatrix->Matrix[0][x][y]);
					}
					else
					{
						fprintf(OutputFile, "%.5f\t%.5f\t%s\n", Coordinate_X, Coordinate_Y, "N.C.");
					}
				}
			}

			/* Close file. */
			fclose(OutputFile);
		}
		else
		{
			printf("\nCould not open the File: %s for writing.\n", Filename);
		}
	}
}

void AntennaPropertiesSet(
	WinProp_Antenna* Antenna,
	double CoordinateX,
	double CoordinateY,
	double Height,
	double Frequency,
	char* Name,
	double Power,
	int PowerMode,
	int Model,
	double Azimuth,
	double Downtilt,
	int Id)
{
	Antenna->Longitude_X = CoordinateX;
	Antenna->Latitude_Y = CoordinateY;
	Antenna->Height = Height;
	Antenna->Power = Power;
	Antenna->PowerMode = PowerMode;
	Antenna->Frequency = Frequency;
	Antenna->Name = Name;
	Antenna->Model = Model;
	Antenna->Azimuth = Azimuth;
	Antenna->Downtilt = Downtilt;
	Antenna->Id = Id;
}

void CarrierPropertiesSet(WinProp_Antenna* Antenna, const WinProp_Carrier* Carrier) {

	Antenna->Carriers.CarrierID = Carrier->CarrierID;
	Antenna->Carriers.SystemID = Carrier->SystemID;
	Antenna->Carriers.CellLoad = Carrier->CellLoad;
	Antenna->Carriers.MimoID = Carrier->MimoID;

	Antenna->Carriers.NoiseRiseUL = Carrier->NoiseRiseUL;
	Antenna->Carriers.PowerBackoffPilotDL = Carrier->PowerBackoffPilotDL;

	// 5G Carrier parameters
	Antenna->Carriers.Numerology = Carrier->Numerology;
	Antenna->Carriers.SymbolsPerSlot = Carrier->SymbolsPerSlot;
	Antenna->Carriers.NrAntennaBeams = Carrier->NrAntennaBeams;
	Antenna->Carriers.BeamGainCtrlServer = Carrier->BeamGainCtrlServer;
	Antenna->Carriers.BeamGainCtrlInterferer = Carrier->BeamGainCtrlInterferer;
	Antenna->Carriers.BeamGainDataServer = Carrier->BeamGainDataServer;
	Antenna->Carriers.BeamGainDataInterferer = Carrier->BeamGainDataInterferer;
	Antenna->Carriers.TDD_Slots_DL = Carrier->TDD_Slots_DL;
	Antenna->Carriers.TDD_Slots_UL = Carrier->TDD_Slots_UL;
	Antenna->Carriers.TDD_Slots_Flex = Carrier->TDD_Slots_Flex;
}