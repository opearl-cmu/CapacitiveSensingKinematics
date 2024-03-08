/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCSTrackingGoal.cpp                                     *
 * -------------------------------------------------------------------------- *
 * Author(s): Owen Pearl                                                      *
 * -------------------------------------------------------------------------- */

#include "MocoCSTrackingGoal.h"

#include <OpenSim/Moco/MocoUtilities.h>

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void MocoCSTrackingGoal::initializeOnModelImpl(const Model&) const {
    
    // Load the TimeSeriesTable
    TimeSeriesTable tableToUse =
            get_reference().process();

    // Turn into Splines
    auto allSplines = GCVSplineSet(tableToUse);

    // Populate the reference spline mutable property (here can add logic if needed)
    for (int iref = 0; iref < allSplines.getSize(); ++iref) {
        m_refsplines.cloneAndAppend(allSplines[iref]);

        // Get column names
        const auto& refName = allSplines[iref].getName();
        m_state_names.push_back(refName);
    }

    // Required set requirements
    setRequirements(1, 1);

}

void MocoCSTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {
    const auto& time = input.time;
    SimTK::Vector timeVec(1, time);
 
    integrand = 0;
    double adjSE = get_refMSE(); // this value can be used to correct the expected error as an offset if you expect a certain level of error in CS predicted FLs

    for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
        // Current value from cs predicted FLs
        const auto& refValue = m_refsplines[iref].calcValue(timeVec);

        // Get muscle fiber length using current column label as muscle name m_state_names[iref]
        const auto& modelValue = getModel().getMuscles().get(m_state_names[iref]).getLength(input.state);

        // Compute the tracking error (can adjust where the optimal squared error is if needed, by default is zero)
        double stateerror = modelValue - refValue;
        double squaredstateerror = stateerror * stateerror;
        double error = squaredstateerror - adjSE;

        // Compute the integrand as the square of the error
        integrand += error * error;
    }
}

void MocoCSTrackingGoal::printDescriptionImpl() const {
    double adjSE = get_refMSE();
    log_cout("        Adjust Cost SE: {}", adjSE);
    for (int i = 0; i < (int) m_state_names.size(); i++) {
        log_cout("        state: {}", m_state_names[i]);
    }
}