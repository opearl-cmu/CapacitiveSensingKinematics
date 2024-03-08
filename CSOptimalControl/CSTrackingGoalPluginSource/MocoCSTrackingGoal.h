#ifndef OPENSIM_MocoCSTrackingGoal_H
#define OPENSIM_MocoCSTrackingGoal_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCSTrackingGoal.h                                            *
 * -------------------------------------------------------------------------- *
 * Author(s): Owen Pearl                                                      *
 * -------------------------------------------------------------------------- */

#include "osimPluginDLL.h"
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Moco/MocoWeightSet.h>
#include <OpenSim/Simulation/TableProcessor.h>

namespace OpenSim {

class OSIMPLUGIN_API MocoCSTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCSTrackingGoal, MocoGoal);

public:
    MocoCSTrackingGoal() { constructProperties(); }
    MocoCSTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoCSTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
    void printDescriptionImpl() const override;

private:
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(reference, TableProcessor,
            "Trajectories of CS data to track"
            "Column labels should be just the nominal name of muscles to which to match to this"
            "e.g., 'soleus_r'");

    OpenSim_DECLARE_PROPERTY(refMSE, double,
            "Reference MSE for adjusting tracking cost with experimental data (0 is no adjustment)");

    void constructProperties() {
            constructProperty_reference(TableProcessor());
            constructProperty_refMSE(0);
    }

    mutable GCVSplineSet m_refsplines;
    mutable std::vector<std::string> m_state_names;

};

} // namespace OpenSim

#endif // OPENSIM_MocoCSTrackingGoal_H