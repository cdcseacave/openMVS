////////////////////////////////////////////////////////////////////
// LBA.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_LBA_H__
#define __SEACAVE_LBA_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define LBP_USE_OPENMP
#endif


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// basic implementation of the loopy belief propagation algorithm
// based on a code originally written by Michael Waechter:
// https://github.com/nmoehrle/mvs-texturing
// Copyright(c) Michael Waechter
// Licensed under the BSD 3-Clause license
class MATH_API LBPInference
{
public:
	typedef unsigned NodeID;
	typedef unsigned LabelID;
	typedef unsigned EdgeID;
	typedef float EnergyType;

	struct DataCost {
		NodeID nodeID;
		EnergyType cost;
	};

	typedef EnergyType (STCALL *FncSmoothCost)(NodeID, NodeID, LabelID, LabelID);

	enum { MaxEnergy = 1000 };

protected:
	struct DirectedEdge {
		NodeID nodeID1;
		NodeID nodeID2;
		std::vector<EnergyType> newMsgs;
		std::vector<EnergyType> oldMsgs;
		inline DirectedEdge(NodeID _nodeID1, NodeID _nodeID2) : nodeID1(_nodeID1), nodeID2(_nodeID2) {}
	};

	struct Node {
		LabelID label;
		EnergyType dataCost;
		std::vector<LabelID> labels;
		std::vector<EnergyType> dataCosts;
		std::vector<EdgeID> incomingEdges;
		inline Node() : label(0), dataCost(MaxEnergy) {}
	};

	std::vector<DirectedEdge> edges;
	std::vector<Node> nodes;
	FncSmoothCost fncSmoothCost;

public:
	LBPInference() {}
	LBPInference(NodeID nNodes) : nodes(nNodes) {}

	inline void SetNumNodes(NodeID nNodes) {
		nodes.resize(nNodes);
	}
	inline NodeID GetNumNodes() const {
		return (NodeID)nodes.size();
	}

	inline void SetNeighbors(NodeID nodeID1, NodeID nodeID2) {
		nodes[nodeID2].incomingEdges.push_back((EdgeID)edges.size());
		edges.push_back(DirectedEdge(nodeID1, nodeID2));
		nodes[nodeID1].incomingEdges.push_back((EdgeID)edges.size());
		edges.push_back(DirectedEdge(nodeID2, nodeID1));
	}

	inline void SetDataCost(LabelID label, NodeID nodeID, EnergyType cost) {
		Node& node = nodes[nodeID];
		node.labels.push_back(label);
		const EnergyType dataCost(cost);
		node.dataCosts.push_back(dataCost);
		if (dataCost < node.dataCost) {
			node.label = label;
			node.dataCost = dataCost;
		}
		for (EdgeID edgeID: node.incomingEdges) {
			DirectedEdge& incomingEdge = edges[edgeID];
			incomingEdge.oldMsgs.push_back(0);
			incomingEdge.newMsgs.push_back(0);
		}
	}
	inline void SetDataCost(LabelID label, const DataCost& cost) {
		SetDataCost(label, cost.nodeID, cost.cost);
	}
	inline void SetDataCosts(LabelID label, const std::vector<DataCost>& costs) {
		for (const DataCost& cost: costs)
			SetDataCost(label, cost);
	}

	inline void SetSmoothCost(FncSmoothCost func) {
		fncSmoothCost = func;
	}

	EnergyType ComputeEnergy() const {
		EnergyType energy(0);
		#ifdef LBP_USE_OPENMP
		#pragma omp parallel for reduction(+:energy)
		#endif
		for (int_t nodeID = 0; nodeID < (int_t)nodes.size(); ++nodeID)
			energy += nodes[nodeID].dataCost;
		#ifdef LBP_USE_OPENMP
		#pragma omp parallel for reduction(+:energy)
		#endif
		for (int_t edgeID = 0; edgeID < (int_t)edges.size(); ++edgeID) {
			const DirectedEdge& edge = edges[edgeID];
			energy += fncSmoothCost(edge.nodeID1, edge.nodeID2, nodes[edge.nodeID1].label, nodes[edge.nodeID2].label);
		}
		return energy;
	}

	void Optimize(unsigned num_iterations) {
		for (unsigned i = 0; i < num_iterations; ++i) {
			#ifdef LBP_USE_OPENMP
			#pragma omp parallel for
			#endif
			for (int_t edgeID = 0; edgeID < (int_t)edges.size(); ++edgeID) {
				DirectedEdge& edge = edges[edgeID];
				const std::vector<LabelID>& labels1 = nodes[edge.nodeID1].labels;
				const std::vector<LabelID>& labels2 = nodes[edge.nodeID2].labels;
				for (size_t j = 0; j < labels2.size(); ++j) {
					const LabelID label2(labels2[j]);
					EnergyType minEnergy(std::numeric_limits<EnergyType>::max());
					for (size_t k = 0; k < labels1.size(); ++k) {
						const LabelID label1(labels1[k]);
						EnergyType energy(nodes[edge.nodeID1].dataCosts[k] + fncSmoothCost(edge.nodeID1, edge.nodeID2, label1, label2));
						const std::vector<EdgeID>& incoming_edges1 = nodes[edge.nodeID1].incomingEdges;
						for (size_t n = 0; n < incoming_edges1.size(); ++n) {
							const DirectedEdge& pre_edge = edges[incoming_edges1[n]];
							if (pre_edge.nodeID1 == edge.nodeID2) continue;
							energy += pre_edge.oldMsgs[k];
						}
						if (minEnergy > energy)
							minEnergy = energy;
					}
					edge.newMsgs[j] = minEnergy;
				}
			}
			#ifdef LBP_USE_OPENMP
			#pragma omp parallel for
			#endif
			for (int_t edgeID = 0; edgeID < (int_t)edges.size(); ++edgeID) {
				DirectedEdge& edge = edges[edgeID];
				edge.newMsgs.swap(edge.oldMsgs);
				EnergyType minMsg(std::numeric_limits<EnergyType>::max());
				for (EnergyType msg: edge.oldMsgs)
					if (minMsg > msg)
						minMsg = msg;
				for (EnergyType& msg: edge.oldMsgs)
					msg -= minMsg;
			}
		}
		#ifdef LBP_USE_OPENMP
		#pragma omp parallel for
		#endif
		for (int_t nodeID = 0; nodeID < (int_t)nodes.size(); ++nodeID) {
			Node& node = nodes[nodeID];
			EnergyType minEnergy(std::numeric_limits<EnergyType>::max());
			for (size_t j = 0; j < node.labels.size(); ++j) {
				EnergyType energy(node.dataCosts[j]);
				for (EdgeID incoming_edge_idx : node.incomingEdges)
					energy += edges[incoming_edge_idx].oldMsgs[j];
				if (energy < minEnergy) {
					minEnergy = energy;
					node.label = node.labels[j];
					node.dataCost = node.dataCosts[j];
				}
			}
		}
	}

	EnergyType Optimize() {
		TD_TIMER_STARTD();
		EnergyType energy(ComputeEnergy());
		EnergyType diff(energy);
		unsigned i(0);
		#if 1
		unsigned nIncreases(0), nTotalIncreases(0);
		#endif
		while (true) {
			TD_TIMER_STARTD();
			const EnergyType last_energy(energy);
			Optimize(1);
			energy = ComputeEnergy();
			diff = last_energy - energy;
			DEBUG_ULTIMATE("\t%2u. e: %g\td: %g\tt: %s", i, last_energy, diff, TD_TIMER_GET_FMT().c_str());
			if (++i > 100 || diff == EnergyType(0))
				break;
			#if 1
			if (diff < EnergyType(0)) {
				++nTotalIncreases;
				if (++nIncreases > 3)
					break;
			} else {
				if (nTotalIncreases > 5)
					break;
				nIncreases = 0;
			}
			#else
			if (diff < EnergyType(0))
				break;
			#endif
		}
		if (diff == EnergyType(0)) {
			DEBUG_ULTIMATE("Inference converged in %u iterations: %g energy (%s)", i, energy, TD_TIMER_GET_FMT().c_str());
		} else {
			DEBUG_ULTIMATE("Inference aborted (energy increased): %u iterations, %g energy (%s)", i, energy, TD_TIMER_GET_FMT().c_str());
		}
		return energy;
	}

	inline LabelID GetLabel(NodeID nodeID) const {
		return nodes[nodeID].label;
	}
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_LBP_H__
