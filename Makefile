#
# OMNeT++/OMNEST Makefile for locvanet
#
# This file was generated with the command:
#  opp_makemake -f --deep -O out -I../veins-RSSI/src -L../veins-RSSI/out/$$\(CONFIGNAME\)/src -lveins-RSSI -KVEINS_RSSI_PROJ=../veins-RSSI
#

# Name of target to be created (-o option)
TARGET = locvanet$(EXE_SUFFIX)

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(QTENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)
#USERIF_LIBS = $(QTENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I../veins-RSSI/src \
    -I. \
    -Icommunication \
    -Ilocalization \
    -Ilocalization/GeographicLib \
    -Ilocalization/GeographicLib/doc \
    -Ilocalization/GeographicLib/include \
    -Ilocalization/GeographicLib/include/GeographicLib \
    -Ilocalization/GeographicLib/src \
    -Ilocalization/jama125 \
    -Ilocalization/tnt_126 \
    -Isimulations \
    -Isimulations/FSPM \
    -Isimulations/RealDist \
    -Isimulations/TRGI \
    -Isimulations/results \
    -Isumoscenarios

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = -L../veins-RSSI/out/$(CONFIGNAME)/src  -lveins-RSSI
LIBS += -Wl,-rpath,`abspath ../veins-RSSI/out/$(CONFIGNAME)/src`

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc, .msg and .sm files
OBJS = \
    $O/communication/LocAppCom.o \
    $O/localization/GeographicLib/src/GeodesicLineExact.o \
    $O/localization/GeographicLib/src/CircularEngine.o \
    $O/localization/GeographicLib/src/DMS.o \
    $O/localization/GeographicLib/src/Gnomonic.o \
    $O/localization/GeographicLib/src/Ellipsoid.o \
    $O/localization/GeographicLib/src/Utility.o \
    $O/localization/GeographicLib/src/Geohash.o \
    $O/localization/GeographicLib/src/GeodesicExact.o \
    $O/localization/GeographicLib/src/GeodesicExactC4.o \
    $O/localization/GeographicLib/src/EllipticFunction.o \
    $O/localization/GeographicLib/src/GeodesicLine.o \
    $O/localization/GeographicLib/src/MagneticModel.o \
    $O/localization/GeographicLib/src/SphericalEngine.o \
    $O/localization/GeographicLib/src/LocalCartesian.o \
    $O/localization/GeographicLib/src/GravityModel.o \
    $O/localization/GeographicLib/src/TransverseMercator.o \
    $O/localization/GeographicLib/src/NormalGravity.o \
    $O/localization/GeographicLib/src/GravityCircle.o \
    $O/localization/GeographicLib/src/Rhumb.o \
    $O/localization/GeographicLib/src/AzimuthalEquidistant.o \
    $O/localization/GeographicLib/src/MGRS.o \
    $O/localization/GeographicLib/src/GeoCoords.o \
    $O/localization/GeographicLib/src/LambertConformalConic.o \
    $O/localization/GeographicLib/src/PolarStereographic.o \
    $O/localization/GeographicLib/src/MagneticCircle.o \
    $O/localization/GeographicLib/src/Geocentric.o \
    $O/localization/GeographicLib/src/CassiniSoldner.o \
    $O/localization/GeographicLib/src/OSGB.o \
    $O/localization/GeographicLib/src/PolygonArea.o \
    $O/localization/GeographicLib/src/Geoid.o \
    $O/localization/GeographicLib/src/AlbersEqualArea.o \
    $O/localization/GeographicLib/src/TransverseMercatorExact.o \
    $O/localization/GeographicLib/src/UTMUPS.o \
    $O/localization/GeographicLib/src/Math.o \
    $O/localization/GeographicLib/src/Geodesic.o \
    $O/localization/GeographicLib/src/GARS.o \
    $O/localization/GeographicLib/src/Accumulator.o \
    $O/localization/GeographicLib/src/Georef.o

# Message files
MSGFILES =

# SM files
SMFILES =

# Other makefile variables (-K)
VEINS_RSSI_PROJ=../veins-RSSI

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppmain$D $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)
SMCOPTS =

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

.PHONY: all clean cleanall depend msgheaders smheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

%_sm.cc %_sm.h: %.sm
	$(qecho) SMC: $<
	$(Q)$(SMC) -c++ -suffix cc $(SMCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

smheaders: $(SMFILES:.sm=_sm.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f locvanet locvanet.exe liblocvanet.so liblocvanet.a liblocvanet.dll liblocvanet.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h ./*_sm.cc ./*_sm.h
	$(Q)-rm -f communication/*_m.cc communication/*_m.h communication/*_sm.cc communication/*_sm.h
	$(Q)-rm -f localization/*_m.cc localization/*_m.h localization/*_sm.cc localization/*_sm.h
	$(Q)-rm -f localization/GeographicLib/*_m.cc localization/GeographicLib/*_m.h localization/GeographicLib/*_sm.cc localization/GeographicLib/*_sm.h
	$(Q)-rm -f localization/GeographicLib/doc/*_m.cc localization/GeographicLib/doc/*_m.h localization/GeographicLib/doc/*_sm.cc localization/GeographicLib/doc/*_sm.h
	$(Q)-rm -f localization/GeographicLib/include/*_m.cc localization/GeographicLib/include/*_m.h localization/GeographicLib/include/*_sm.cc localization/GeographicLib/include/*_sm.h
	$(Q)-rm -f localization/GeographicLib/include/GeographicLib/*_m.cc localization/GeographicLib/include/GeographicLib/*_m.h localization/GeographicLib/include/GeographicLib/*_sm.cc localization/GeographicLib/include/GeographicLib/*_sm.h
	$(Q)-rm -f localization/GeographicLib/src/*_m.cc localization/GeographicLib/src/*_m.h localization/GeographicLib/src/*_sm.cc localization/GeographicLib/src/*_sm.h
	$(Q)-rm -f localization/jama125/*_m.cc localization/jama125/*_m.h localization/jama125/*_sm.cc localization/jama125/*_sm.h
	$(Q)-rm -f localization/tnt_126/*_m.cc localization/tnt_126/*_m.h localization/tnt_126/*_sm.cc localization/tnt_126/*_sm.h
	$(Q)-rm -f simulations/*_m.cc simulations/*_m.h simulations/*_sm.cc simulations/*_sm.h
	$(Q)-rm -f simulations/FSPM/*_m.cc simulations/FSPM/*_m.h simulations/FSPM/*_sm.cc simulations/FSPM/*_sm.h
	$(Q)-rm -f simulations/RealDist/*_m.cc simulations/RealDist/*_m.h simulations/RealDist/*_sm.cc simulations/RealDist/*_sm.h
	$(Q)-rm -f simulations/TRGI/*_m.cc simulations/TRGI/*_m.h simulations/TRGI/*_sm.cc simulations/TRGI/*_sm.h
	$(Q)-rm -f simulations/results/*_m.cc simulations/results/*_m.h simulations/results/*_sm.cc simulations/results/*_sm.h
	$(Q)-rm -f sumoscenarios/*_m.cc sumoscenarios/*_m.h sumoscenarios/*_sm.cc sumoscenarios/*_sm.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES) $(SM_CC_FILES)  ./*.cc communication/*.cc localization/*.cc localization/GeographicLib/*.cc localization/GeographicLib/doc/*.cc localization/GeographicLib/include/*.cc localization/GeographicLib/include/GeographicLib/*.cc localization/GeographicLib/src/*.cc localization/jama125/*.cc localization/tnt_126/*.cc simulations/*.cc simulations/FSPM/*.cc simulations/RealDist/*.cc simulations/TRGI/*.cc simulations/results/*.cc sumoscenarios/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/communication/LocAppCom.o: communication/LocAppCom.cc \
	communication/LocAppCom.h \
	localization/jama125/jama_qr.h \
	localization/tnt_126/tnt_array1d.h \
	localization/tnt_126/tnt_array2d.h \
	localization/tnt_126/tnt_i_refvec.h \
	localization/tnt_126/tnt_math_utils.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/connectionManager/BaseConnectionManager.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/connectionManager/ChannelAccess.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/connectionManager/NicEntry.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/modules/BaseApplLayer.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/modules/BaseBattery.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/modules/BaseLayer.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/modules/BaseMobility.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/modules/BaseModule.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/modules/BaseWorldUtility.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/modules/BatteryAccess.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/FindModule.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/HostState.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/Move.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/NetwToMacControlInfo.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/PassedMessage.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/SimpleAddress.h \
	$(VEINS_RSSI_PROJ)/src/veins/base/utils/miximkerneldefs.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/messages/WaveShortMessage_m.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/mobility/traci/TraCIBuffer.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/mobility/traci/TraCIColor.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/mobility/traci/TraCICommandInterface.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/mobility/traci/TraCIConnection.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/mobility/traci/TraCICoord.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/mobility/traci/TraCIMobility.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/mobility/traci/TraCIScenarioManager.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/obstacle/Obstacle.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/obstacle/ObstacleControl.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/utility/Consts80211p.h \
	$(VEINS_RSSI_PROJ)/src/veins/modules/world/annotations/AnnotationManager.h
$O/localization/GeographicLib/src/Accumulator.o: localization/GeographicLib/src/Accumulator.cc
$O/localization/GeographicLib/src/AlbersEqualArea.o: localization/GeographicLib/src/AlbersEqualArea.cc
$O/localization/GeographicLib/src/AzimuthalEquidistant.o: localization/GeographicLib/src/AzimuthalEquidistant.cc
$O/localization/GeographicLib/src/CassiniSoldner.o: localization/GeographicLib/src/CassiniSoldner.cc
$O/localization/GeographicLib/src/CircularEngine.o: localization/GeographicLib/src/CircularEngine.cc
$O/localization/GeographicLib/src/DMS.o: localization/GeographicLib/src/DMS.cc
$O/localization/GeographicLib/src/Ellipsoid.o: localization/GeographicLib/src/Ellipsoid.cc
$O/localization/GeographicLib/src/EllipticFunction.o: localization/GeographicLib/src/EllipticFunction.cc
$O/localization/GeographicLib/src/GARS.o: localization/GeographicLib/src/GARS.cc
$O/localization/GeographicLib/src/GeoCoords.o: localization/GeographicLib/src/GeoCoords.cc
$O/localization/GeographicLib/src/Geocentric.o: localization/GeographicLib/src/Geocentric.cc
$O/localization/GeographicLib/src/Geodesic.o: localization/GeographicLib/src/Geodesic.cc
$O/localization/GeographicLib/src/GeodesicExact.o: localization/GeographicLib/src/GeodesicExact.cc
$O/localization/GeographicLib/src/GeodesicExactC4.o: localization/GeographicLib/src/GeodesicExactC4.cc
$O/localization/GeographicLib/src/GeodesicLine.o: localization/GeographicLib/src/GeodesicLine.cc
$O/localization/GeographicLib/src/GeodesicLineExact.o: localization/GeographicLib/src/GeodesicLineExact.cc
$O/localization/GeographicLib/src/Geohash.o: localization/GeographicLib/src/Geohash.cc
$O/localization/GeographicLib/src/Geoid.o: localization/GeographicLib/src/Geoid.cc
$O/localization/GeographicLib/src/Georef.o: localization/GeographicLib/src/Georef.cc
$O/localization/GeographicLib/src/Gnomonic.o: localization/GeographicLib/src/Gnomonic.cc
$O/localization/GeographicLib/src/GravityCircle.o: localization/GeographicLib/src/GravityCircle.cc
$O/localization/GeographicLib/src/GravityModel.o: localization/GeographicLib/src/GravityModel.cc
$O/localization/GeographicLib/src/LambertConformalConic.o: localization/GeographicLib/src/LambertConformalConic.cc
$O/localization/GeographicLib/src/LocalCartesian.o: localization/GeographicLib/src/LocalCartesian.cc
$O/localization/GeographicLib/src/MGRS.o: localization/GeographicLib/src/MGRS.cc
$O/localization/GeographicLib/src/MagneticCircle.o: localization/GeographicLib/src/MagneticCircle.cc
$O/localization/GeographicLib/src/MagneticModel.o: localization/GeographicLib/src/MagneticModel.cc
$O/localization/GeographicLib/src/Math.o: localization/GeographicLib/src/Math.cc
$O/localization/GeographicLib/src/NormalGravity.o: localization/GeographicLib/src/NormalGravity.cc
$O/localization/GeographicLib/src/OSGB.o: localization/GeographicLib/src/OSGB.cc
$O/localization/GeographicLib/src/PolarStereographic.o: localization/GeographicLib/src/PolarStereographic.cc
$O/localization/GeographicLib/src/PolygonArea.o: localization/GeographicLib/src/PolygonArea.cc
$O/localization/GeographicLib/src/Rhumb.o: localization/GeographicLib/src/Rhumb.cc
$O/localization/GeographicLib/src/SphericalEngine.o: localization/GeographicLib/src/SphericalEngine.cc
$O/localization/GeographicLib/src/TransverseMercator.o: localization/GeographicLib/src/TransverseMercator.cc
$O/localization/GeographicLib/src/TransverseMercatorExact.o: localization/GeographicLib/src/TransverseMercatorExact.cc
$O/localization/GeographicLib/src/UTMUPS.o: localization/GeographicLib/src/UTMUPS.cc
$O/localization/GeographicLib/src/Utility.o: localization/GeographicLib/src/Utility.cc

