#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for(int i=0;i<num_nodes;i++)
        {
            Vector2D position=start+i*(end-start)/(num_nodes-1);
            Mass* mass=new Mass(position,node_mass,false);
            masses.emplace_back(mass);
            if(i>0)
            {
                Spring* spring=new Spring(masses[i-1],masses[i],k);
                springs.emplace_back(spring);
            }
        }
//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
for(auto &i:pinned_nodes)
{
        masses[i]->pinned=true;
}
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D a=s->m1->position;
            Vector2D b=s->m2->position;
            float static_length=s->rest_length;
            float k=s->k;
            Vector2D ba=b-a;
            Vector2D force_ba=k*ba.unit()*(ba.norm()-static_length);
            s->m1->forces+=force_ba;
            s->m2->forces-=force_ba;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces+=gravity*m->mass;
                // TODO (Part 2): Add global damping
                float damping_coefficient=0.0001;
                m->forces+=-damping_coefficient*m->velocity;
                //explicit euler's method
                //m->position+=m->velocity*delta_t;
                //semi-explicit euler's method
                
                m->velocity+=(m->forces/m->mass)*delta_t;
                m->position+=m->velocity*delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D a=s->m1->position;
            Vector2D b=s->m2->position;
            float static_length=s->rest_length;
            float k=s->k;
            Vector2D ba=b-a;
            Vector2D force_ba=k*ba.unit()*(ba.norm()-static_length);
            s->m1->forces+=force_ba;
            s->m2->forces-=force_ba;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
                m->forces+=gravity*m->mass;
                float dumping_coefficient=0.0005;
                Vector2D a=m->forces/m->mass;
                m->position+=(1-dumping_coefficient)*(m->position-m->last_position)+a*delta_t*delta_t;
                m->last_position=temp_position;
            }
        }
    }
}
