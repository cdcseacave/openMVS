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
// NOTE: deliberately NOT tagged with MATH_API at class level — every method
// is defined inline in this header, so consumers can instantiate them locally.
// Tagging would force MSVC to emit __declspec(dllimport) at call sites and
// expect Math.dll to export them, but Math.dll has no out-of-line definitions.
class LBPInference
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

protected:
	struct DirectedEdge {
		NodeID nodeID1;
		NodeID nodeID2;
		EnergyType weight;
		size_t msgs; // offset of its messages, one per label of nodeID2, in oldMsgs/newMsgs
		inline DirectedEdge(NodeID _nodeID1, NodeID _nodeID2, EnergyType _weight) : nodeID1(_nodeID1), nodeID2(_nodeID2), weight(_weight), msgs(0) {}
	};

	struct Node {
		LabelID label;
		EnergyType dataCost;
		std::vector<LabelID> labels;
		std::vector<EnergyType> dataCosts;
		std::vector<EdgeID> incomingEdges;
	};

	std::vector<DirectedEdge> edges;
	std::vector<Node> nodes;
	// the messages of every directed edge in two flat buffers (one value per label of its target
	// node), allocated once the labels are final, at the first optimization
	std::vector<EnergyType> oldMsgs, newMsgs;
	FncSmoothCost fncSmoothCost;
	// >= 0 when fncSmoothCost is the Potts model with this cost (0 for two equal labels other than
	// label 0, pottsCost otherwise): the messages then take O(labels) instead of O(labels^2)
	EnergyType pottsCost = -1;

public:
	LBPInference() {}
	LBPInference(NodeID nNodes) : nodes(nNodes) {}

	inline void SetNumNodes(NodeID nNodes) {
		nodes.resize(nNodes);
	}
	inline NodeID GetNumNodes() const {
		return (NodeID)nodes.size();
	}

	inline void SetNeighbors(NodeID nodeID1, NodeID nodeID2, EnergyType weight = 1) {
		nodes[nodeID2].incomingEdges.push_back((EdgeID)edges.size());
		edges.push_back(DirectedEdge(nodeID1, nodeID2, weight));
		nodes[nodeID1].incomingEdges.push_back((EdgeID)edges.size());
		edges.push_back(DirectedEdge(nodeID2, nodeID1, weight));
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
		ASSERT(oldMsgs.empty()); // the labels are fixed once the optimization started
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
		pottsCost = -1;
	}
	// func must be the Potts model of the given cost (see pottsCost)
	inline void SetSmoothCostPotts(FncSmoothCost func, EnergyType cost) {
		ASSERT(cost >= 0 && func(0, 0, 1, 1) == 0 && func(0, 0, 1, 2) == cost && func(0, 0, 0, 0) == cost);
		fncSmoothCost = func;
		pottsCost = cost;
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
			energy += fncSmoothCost(edge.nodeID1, edge.nodeID2, nodes[edge.nodeID1].label, nodes[edge.nodeID2].label) * edge.weight;
		}
		return energy;
	}

	void Optimize(unsigned num_iterations) {
		if (oldMsgs.empty()) {
			size_t numMsgs(0);
			for (DirectedEdge& edge: edges) {
				edge.msgs = numMsgs;
				numMsgs += nodes[edge.nodeID2].labels.size();
			}
			oldMsgs.assign(numMsgs, EnergyType(0));
			newMsgs.assign(numMsgs, EnergyType(0));
		}
		for (unsigned i = 0; i < num_iterations; ++i) {
			#ifdef LBP_USE_OPENMP
			#pragma omp parallel for
			#endif
			for (int_t edgeID = 0; edgeID < (int_t)edges.size(); ++edgeID) {
				DirectedEdge& edge = edges[edgeID];
				const std::vector<LabelID>& labels1 = nodes[edge.nodeID1].labels;
				const std::vector<LabelID>& labels2 = nodes[edge.nodeID2].labels;
				if (pottsCost >= 0) {
					// Potts: the energy of sending label k is the same for every receiving label but
					// k's own, which costs the smoothness less; with E0/E1 the energies of k at
					// smoothness 0/pottsCost (summed in the generic loop's order, so the messages
					// are bit-identical), the message of label j is min(E0[k=j], min_k E1[k]):
					// E0[k] <= E1[k] makes the excluded E1[k=j] irrelevant
					const Node& node1 = nodes[edge.nodeID1];
					thread_local std::vector<EnergyType> e0, e1;
					e0.resize(labels1.size());
					e1.resize(labels1.size());
					EnergyType minE1(std::numeric_limits<EnergyType>::max());
					for (size_t k = 0; k < labels1.size(); ++k) {
						EnergyType energy0(node1.dataCosts[k] + EnergyType(0) * edge.weight);
						EnergyType energy1(node1.dataCosts[k] + pottsCost * edge.weight);
						for (EdgeID idxIncomingEdge: node1.incomingEdges) {
							const DirectedEdge& pre_edge = edges[idxIncomingEdge];
							if (pre_edge.nodeID1 == edge.nodeID2)
								continue;
							energy0 += oldMsgs[pre_edge.msgs+k];
							energy1 += oldMsgs[pre_edge.msgs+k];
						}
						e0[k] = energy0;
						e1[k] = energy1;
						if (minE1 > energy1)
							minE1 = energy1;
					}
					for (size_t j = 0; j < labels2.size(); ++j) {
						const LabelID label2(labels2[j]);
						EnergyType minEnergy(minE1);
						if (label2 != 0) {
							for (size_t k = 0; k < labels1.size(); ++k) {
								if (labels1[k] == label2) {
									if (minEnergy > e0[k])
										minEnergy = e0[k];
									break;
								}
							}
						}
						newMsgs[edge.msgs+j] = minEnergy;
					}
					continue;
				}
				for (size_t j = 0; j < labels2.size(); ++j) {
					const LabelID label2(labels2[j]);
					EnergyType minEnergy(std::numeric_limits<EnergyType>::max());
					for (size_t k = 0; k < labels1.size(); ++k) {
						const LabelID label1(labels1[k]);
						EnergyType energy(nodes[edge.nodeID1].dataCosts[k] + fncSmoothCost(edge.nodeID1, edge.nodeID2, label1, label2) * edge.weight);
						for (EdgeID idxIncomingEdge: nodes[edge.nodeID1].incomingEdges) {
							const DirectedEdge& pre_edge = edges[idxIncomingEdge];
							if (pre_edge.nodeID1 == edge.nodeID2)
								continue;
							energy += oldMsgs[pre_edge.msgs+k];
						}
						if (minEnergy > energy)
							minEnergy = energy;
					}
					newMsgs[edge.msgs+j] = minEnergy;
				}
			}
			newMsgs.swap(oldMsgs);
			#ifdef LBP_USE_OPENMP
			#pragma omp parallel for
			#endif
			for (int_t edgeID = 0; edgeID < (int_t)edges.size(); ++edgeID) {
				const DirectedEdge& edge = edges[edgeID];
				EnergyType* const msgs(oldMsgs.data()+edge.msgs);
				const size_t numLabels(nodes[edge.nodeID2].labels.size());
				EnergyType minMsg(std::numeric_limits<EnergyType>::max());
				for (size_t j = 0; j < numLabels; ++j)
					if (minMsg > msgs[j])
						minMsg = msgs[j];
				for (size_t j = 0; j < numLabels; ++j)
					msgs[j] -= minMsg;
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
				for (EdgeID idxIncomingEdge: node.incomingEdges)
					energy += oldMsgs[edges[idxIncomingEdge].msgs+j];
				if (minEnergy > energy) {
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
