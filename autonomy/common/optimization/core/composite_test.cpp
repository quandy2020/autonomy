/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/common/optimization/core/composite.hpp"

#include <gtest/gtest.h>

namespace autonomy {
namespace common {
namespace optimization {

class ExComponent : public Component
{
public:
    ExComponent(int n_var, const std::string& name) : Component(n_var, name) {};

    virtual VectorXd GetValues() const {
        throw std::runtime_error("not implemented");
    };
    virtual VecBound GetBounds() const {
        throw std::runtime_error("not implemented");
    };
    virtual Jacobian GetJacobian() const {
        throw std::runtime_error("not implemented");
    };
    virtual void SetVariables(const VectorXd& x) {};
};

TEST(Component, GetRows) {
    ExComponent c(2, "ex_component");
    EXPECT_EQ(2, c.GetRows());

    c.SetRows(4);
    EXPECT_EQ(4, c.GetRows());
}

TEST(Component, GetName) {
    ExComponent c(2, "ex_component");
    EXPECT_STREQ("ex_component", c.GetName().c_str());
}

TEST(Composite, GetRowsCost) {
    auto c1 = std::make_shared<ExComponent>(0, "component1");
    auto c2 = std::make_shared<ExComponent>(1, "component2");
    auto c3 = std::make_shared<ExComponent>(2, "component3");

    Composite cost("cost", true);
    cost.AddComponent(c1);
    cost.AddComponent(c2);
    cost.AddComponent(c3);
    EXPECT_EQ(1, cost.GetRows());
}

TEST(Composite, GetRowsConstraint) {
    auto c1 = std::make_shared<ExComponent>(0, "component1");
    auto c2 = std::make_shared<ExComponent>(1, "component2");
    auto c3 = std::make_shared<ExComponent>(2, "component3");

    Composite constraint("constraint", false);
    constraint.AddComponent(c1);
    constraint.AddComponent(c2);
    constraint.AddComponent(c3);
    EXPECT_EQ(0 + 1 + 2, constraint.GetRows());
}

TEST(Composite, GetComponent) {
    auto c1 = std::make_shared<ExComponent>(0, "component1");
    auto c2 = std::make_shared<ExComponent>(1, "component2");
    auto c3 = std::make_shared<ExComponent>(2, "component3");

    Composite comp("composite", false);
    comp.AddComponent(c1);
    comp.AddComponent(c2);
    comp.AddComponent(c3);

    auto c1_new = comp.GetComponent("component1");
    EXPECT_EQ(c1->GetRows(), c1_new->GetRows());

    auto c2_new = comp.GetComponent<ExComponent>("component2");
    EXPECT_EQ(c2->GetRows(), c2_new->GetRows());

    auto c3_new = comp.GetComponent<ExComponent>("component3");
    EXPECT_NE(c1->GetRows(), c3_new->GetRows());
}

TEST(Composite, ClearComponents) {
    auto c1 = std::make_shared<ExComponent>(0, "component1");
    auto c2 = std::make_shared<ExComponent>(1, "component2");
    auto c3 = std::make_shared<ExComponent>(2, "component3");

    Composite comp("composite", false);
    comp.AddComponent(c1);
    comp.AddComponent(c2);
    comp.AddComponent(c3);

    EXPECT_EQ(0 + 1 + 2, comp.GetRows());

    comp.ClearComponents();
    EXPECT_EQ(0, comp.GetRows());
}

}  // namespace optimization
}  // namespace common
}  // namespace autonomy