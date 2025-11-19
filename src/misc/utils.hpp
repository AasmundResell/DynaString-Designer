#pragma once
#include "includes.hpp"
#include "sim-tools/arena/arena.hpp"
#include "sim-tools/timer.hpp"

#include <cstdarg>
using Timer = sito::Timer;

constexpr scalar STANDARD_GRAVITY = 9.80665;

enum class SimulationMode {
    MAX_STEPS,
    MAX_TIME,
    INFINITE_LOOP
};

enum class StringType {
    DRILL_STRING,
    CASING_STRING,
    COILED_TUBING
};

const map<string, StringType> string_type_from_string{{"drill_string", StringType::DRILL_STRING},
                                                      {"casing_string", StringType::CASING_STRING},
                                                      {"coiled_tubing", StringType::COILED_TUBING}};

enum class ComponentType {
    NONE = 0,
    DRILL_PIPE,
    STABILIZER,
    CASING,
    STEERING,
    MWD,
    SUB,
    COUNT
};

const map<string, ComponentType> component_type_from_string_user_input{{"drill_pipe", ComponentType::DRILL_PIPE},
                                                                       {"hwdp", ComponentType::DRILL_PIPE},
                                                                       {"drill_collar", ComponentType::DRILL_PIPE},
                                                                       {"stabilizer", ComponentType::STABILIZER},
                                                                       {"casing", ComponentType::CASING},
                                                                       {"liner", ComponentType::CASING},
                                                                       {"mwd", ComponentType::MWD},
                                                                       {"lwd", ComponentType::MWD},
                                                                       {"pwd", ComponentType::MWD},
                                                                       {"jar", ComponentType::SUB},
                                                                       {"sub", ComponentType::SUB},
                                                                       {"cross-over", ComponentType::SUB},
                                                                       {"float_sub", ComponentType::SUB},
                                                                       {"asm", ComponentType::SUB},
                                                                       {"rss", ComponentType::STEERING},
                                                                       {"bent-housing", ComponentType::STEERING},
                                                                       {"push-the-bit", ComponentType::STEERING}};

constexpr std::array<const char *, static_cast<size_t>(ComponentType::COUNT)> base_string_from_component_type = {
    "Unspecified", "Drill Pipe", "Stabilizer", "Casing", "Steering Tool", "MWD", "Sub"};

enum class SteeringMode {
    NONE,
    BENT_HOUSING,
    RSS,
    PUSH_THE_BIT
};

const map<string, SteeringMode> steering_mode_from_string{{"none", SteeringMode::NONE},
                                                          {"bent_housing", SteeringMode::BENT_HOUSING},
                                                          {"rss", SteeringMode::RSS},
                                                          {"push_the_bit", SteeringMode::PUSH_THE_BIT}};

enum class CentralizerType {
    BOW_SPRING = 0,
    RIGID
};

enum class LengthUnit {
    METER,
    INCH
};

const map<string, LengthUnit> length_unit_from_string{{"meter", LengthUnit::METER}, {"inch", LengthUnit::INCH}};

enum class PipeSolverType {
    CURVILINEAR,
    COROTATIONAL
};
const map<string, PipeSolverType> pipe_solver_type_from_string{{"curvilinear", PipeSolverType::CURVILINEAR},
                                                               {"corotational", PipeSolverType::COROTATIONAL}};

enum class HoleProfileType {
    LINEAR,
    VECTOR
};

const map<string, HoleProfileType> hole_profile_type_from_string{{"linear", HoleProfileType::LINEAR},
                                                                 {"vector", HoleProfileType::VECTOR}};

enum class HoleTrajectoryType {
    MINIMUM_CURVATURE,
    CONTINUOUS_CURVATURE
};

const map<string, HoleTrajectoryType> hole_trajectory_build_type_from_string{
    {"minimum-curvature", HoleTrajectoryType::MINIMUM_CURVATURE},
    {"continuous-curvature", HoleTrajectoryType::CONTINUOUS_CURVATURE}};

enum class HoleSurfaceType {
    CIRCULAR,
    ELLIPTICAL
};

const map<string, HoleSurfaceType> hole_surface_type_from_string{{"circular", HoleSurfaceType::CIRCULAR},
                                                                 {"elliptical", HoleSurfaceType::ELLIPTICAL}};

enum class HoleOffsetBuildType {
    NONE,
    ANALYTICAL_SPIRAL,
    RANDOM_FIELD,
};

const map<string, HoleOffsetBuildType> hole_offset_build_type_from_string{
    {"none", HoleOffsetBuildType::NONE},
    {"analytical-spiral", HoleOffsetBuildType::ANALYTICAL_SPIRAL},
    {"random-field", HoleOffsetBuildType::RANDOM_FIELD}};

enum class HoleCrossSectionBuildType {
    CONSTANT,
    RANDOM_FIELD,
};

const map<string, HoleCrossSectionBuildType> hole_cross_section_build_type_from_string{
    {"constant", HoleCrossSectionBuildType::CONSTANT}, {"random-field", HoleCrossSectionBuildType::RANDOM_FIELD}};

enum class CorotationalFormulation {
    CRISFIELD,
    BATTINI,
    COUNT
};

const map<string, CorotationalFormulation> corotational_formulation_from_string{
    {"crisfield", CorotationalFormulation::CRISFIELD}, {"battini", CorotationalFormulation::BATTINI}};

enum class CurvilinearIntegrationType {
    GAUSS_LEGENDRE,
    PRE_DERIVED,
    SOFT_STRING,
    COUNT
};

const map<string, CurvilinearIntegrationType> curvilinear_integration_type_from_string{
    {"prederived", CurvilinearIntegrationType::PRE_DERIVED},
    {"gauss-legendre", CurvilinearIntegrationType::GAUSS_LEGENDRE},
    {"soft-string", CurvilinearIntegrationType::SOFT_STRING}};

enum class BC_BottomAxialKinematics {
    FREE,
    BOTTOM_CONTACT,
    FIXED,
    COUNT
};

const map<string, BC_BottomAxialKinematics> bc_axial_kinematics_from_string{
    {"free", BC_BottomAxialKinematics::FREE},
    {"bottom-contact", BC_BottomAxialKinematics::BOTTOM_CONTACT},
    {"fixed", BC_BottomAxialKinematics::FIXED},
};

enum class BC_TopKinematicsType {
    DIRICHLET,
    NEUMANN,
    COUNT
};

const map<string, BC_TopKinematicsType> bc_top_kinematics_from_string{
    {"dirichlet", BC_TopKinematicsType::DIRICHLET},
    {"neumann", BC_TopKinematicsType::NEUMANN},
};

enum class BC_BitRockType {
    DETOURNAY,
    DETOURNAY_REGULARIZED,
    TUCKER_WANG,
    TUCKER_WANG_TORSIONAL,
    NO_BIT, // Disables digging
    COUNT
};

const map<string, BC_BitRockType> bc_bit_rock_type_from_string{
    {"detournay", BC_BitRockType::DETOURNAY},
    {"detournay-regularized", BC_BitRockType::DETOURNAY_REGULARIZED},
    {"tucker-wang", BC_BitRockType::TUCKER_WANG},
    {"tucker-wang-torsional", BC_BitRockType::TUCKER_WANG_TORSIONAL},
    {"none", BC_BitRockType::NO_BIT},
};

enum class BC_TopOmegaControllerType {
    PID, // soft-speed -> PID specific controller gains
    SOFT_TORQUE,
    Z_TORQUE,
    COUNT
};

const map<string, BC_TopOmegaControllerType> bc_top_omega_type_from_string{
    {"pid", BC_TopOmegaControllerType::PID},
    {"soft-torque", BC_TopOmegaControllerType::SOFT_TORQUE},
    {"z-torque", BC_TopOmegaControllerType::Z_TORQUE},
};

enum class FSI_ForceModel {
    NONE,
    JANSEN,
    FRITZ,
    PAIDOUSSIS,
    RESELL
};

const map<string, FSI_ForceModel> fsi_force_model_from_string{{"none", FSI_ForceModel::NONE},
                                                              {"jansen", FSI_ForceModel::JANSEN},
                                                              {"fritz", FSI_ForceModel::FRITZ},
                                                              {"paidoussis", FSI_ForceModel::PAIDOUSSIS}};

enum class FSI_TorqueModel {
    NONE,
    CY, // Christoforou & Yigit
    RESELL
};

const map<string, FSI_TorqueModel> fsi_torque_model_from_string{{"none", FSI_TorqueModel::NONE},
                                                                {"cy", FSI_TorqueModel::CY}};

struct CubicSplineCoeffs {
    /*The coefficients defines the displacement in each node as u(t) = a*t³ + b*t² + c*t + d
    These should be updated externally to model an appropriate disp field up to cubic order*/
    Vec3 a, b, c, d;
    friend ostream &operator<<(ostream &os, const CubicSplineCoeffs &val) {
        os << "a: " << val.a.transpose() << ", b: " << val.b.transpose() << ", c: " << val.c.transpose() << "\n";
        return os;
    }
    DEVICE_FUNC Vec3 calc_u(scalar t) const { return a * powi(t, 3) + b * powi(t, 2) + c * t + d; }
    DEVICE_FUNC Vec3 calc_du_dt(scalar t) const { return 3 * a * powi(t, 2) + 2 * b * t + c; }
    DEVICE_FUNC Vec3 calc_d2u_dt2(scalar t) const { return 6 * a * t + 2 * b; }
};
