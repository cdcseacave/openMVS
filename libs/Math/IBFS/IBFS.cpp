/*
#########################################################
#                                                       #
#  IBFSGraph -  Software for solving                    #
#               Maximum s-t Flow / Minimum s-t Cut      #
#               using the IBFS algorithm                #
#                                                       #
#  http://www.cs.tau.ac.il/~sagihed/ibfs/               #
#                                                       #
#  Haim Kaplan (haimk@cs.tau.ac.il)                     #
#  Sagi Hed (sagihed@post.tau.ac.il)                    #
#                                                       #
#########################################################

This software implements the IBFS (Incremental Breadth First Search) maximum flow algorithm from
	"Maximum flows by incremental breadth-first search"
	Andrew V. Goldberg, Sagi Hed, Haim Kaplan, Robert E. Tarjan, and Renato F. Werneck.
	In Proceedings of the 19th European conference on Algorithms, ESA'11, pages 457-468.
	ISBN 978-3-642-23718-8
	2011

Copyright Haim Kaplan (haimk@cs.tau.ac.il) and Sagi Hed (sagihed@post.tau.ac.il)

###########
# LICENSE #
###########
This software can be used for research purposes only.
If you use this software for research purposes, you should cite the aforementioned paper
in any resulting publication and appropriately credit it.

If you require another license, please contact the above.

*/

#include "Common.h"
#include "IBFS.h"

using namespace IBFS;


//
// Orphan handling
//
#define ADD_ORPHAN_BACK(n)							\
if (orphanFirst != IB_ORPHANS_END)					\
{													\
	orphanLast = (orphanLast->nextPtr = (n));		\
}													\
else												\
{													\
	orphanLast = (orphanFirst = (n));				\
}													\
(n)->nextPtr = IB_ORPHANS_END





#define ADD_ORPHAN_FRONT(n)							\
if (orphanFirst == IB_ORPHANS_END)					\
{													\
	(n)->nextPtr = IB_ORPHANS_END;					\
	orphanLast = (orphanFirst = (n));				\
}													\
else												\
{													\
	(n)->nextPtr = orphanFirst;						\
	orphanFirst = (n);								\
}




IBFSGraph::IBFSGraph()
{
	numNodes = 0;
	uniqOrphansS = uniqOrphansT = 0;
	augTimestamp = 0;
	verbose = IBTEST;
	compactSlowInitMode = false;
	arcs = arcEnd = NULL;
	nodes = nodeEnd = NULL;
	topLevelS = topLevelT = 0;
	flow = 0;
	orphanFirst = orphanLast = NULL;
	memArcs = NULL;
	tmpArcs = NULL;
	tmpEdges = tmpEdgeLast = NULL;
}


IBFSGraph::~IBFSGraph()
{
	active0.release();
	activeS1.release();
	activeT1.release();
	orphanBuckets.release();
	delete[] memArcs;
	delete[] nodes;
}

void IBFSGraph::initGraph()
{
	if (compactSlowInitMode) {
		initGraphCompact();
	} else {
		initGraphFast();
	}
}


void IBFSGraph::initSize(int numNodes, int numEdges)
{
	// allocate nodes
	if (verbose) {
		fprintf(stdout, "c allocating nodes... \t [%zu]\n", sizeof(Node)*(numNodes+1));
		fflush(stdout);
	}
	this->numNodes = numNodes;
	nodes = new Node[numNodes+1];
	memset(nodes, 0, sizeof(Node)*(numNodes+1));
	nodeEnd = nodes+numNodes;
	active0.init(numNodes);
	activeS1.init(numNodes);
	activeT1.init(numNodes);
	orphanBuckets.init(nodes, numNodes);

	// allocate arcs
	size_t arcMemsize = sizeof(TmpArc)*(numEdges*2) + sizeof(TmpEdge)*numEdges;
	if (arcMemsize < sizeof(Arc)*(numEdges*2)) {
		arcMemsize = sizeof(Arc)*(numEdges*2);
	}
	if (verbose) {
		fprintf(stdout, "c allocating arcs... \t [%zu]\n", arcMemsize);
		fflush(stdout);
	}
	memArcs = new char[arcMemsize];
	memset(memArcs, 0, sizeof(char)*arcMemsize);
	tmpEdges = (TmpEdge*)(memArcs);
	tmpEdgeLast = tmpEdges; // will advance as edges are added
	tmpArcs = (TmpArc*)(memArcs +sizeof(TmpEdge)*numEdges);
	arcs = (Arc*)memArcs;
	arcEnd = arcs + numEdges*2;

	// init members
	flow = 0;

	if (verbose) {
		fprintf(stdout, "c sizeof(ptr) = %zu bytes\n", sizeof(Node*));
		fprintf(stdout, "c sizeof(node) = %zu bytes\n", sizeof(Node));
		fprintf(stdout, "c sizeof(arc) = %zu bytes\n", sizeof(Arc));
		fprintf(stdout, "c #nodes = %zu \n", nodeEnd-nodes);
		fprintf(stdout, "c #arcs = %zu \n", (arcEnd-arcs) + (nodeEnd-nodes));
		fprintf(stdout, "c #grid_arcs = %zu \n", arcEnd-arcs);
	}
}


void IBFSGraph::initGraphFast()
{
	Node *x;
	Arc *a;
	TmpArc *ta, *taEnd;
	TmpEdge *te;

	// tmpEdges:			edges read
	// node.label:			out degree

	// calculate start arc offsets every node
	nodes->firstArc = (Arc*)(tmpArcs);
	for (x=nodes; x != nodeEnd; x++) {
		(x+1)->firstArc = (Arc*)(((TmpArc*)(x->firstArc)) + x->label);
		x->label = (int)(((TmpArc*)(x->firstArc))-tmpArcs);
	}
	nodeEnd->label = (int)(arcEnd-arcs);

	// tmpEdges:				edges read
	// node.label: 				index into arcs array of first out arc
	// node.firstArc-tmpArcs: 	index into arcs array of next out arc to be allocated
	//							(initially the first out arc)

	// copy to temp arcs memory
	if (verbose) {
		IBDEBUG("c initFast copy1");
	}
	for (te=tmpEdges; te != tmpEdgeLast; te++) {
		ta = (TmpArc*)(te->tail->firstArc);
		ta->cap = te->cap;
		ta->rev = (TmpArc*)(te->head->firstArc);

		ta = (TmpArc*)(te->head->firstArc);
		ta->cap = te->revCap;
		ta->rev = (TmpArc*)(te->tail->firstArc);

		te->tail->firstArc = (Arc*)(((TmpArc*)(te->tail->firstArc))+1);
		te->head->firstArc = (Arc*)(((TmpArc*)(te->head->firstArc))+1);
	}

	// tmpEdges:				edges read
	// tmpArcs:					arcs with reverse pointer but no node id
	// node.label: 				index into arcs array of first out arc
	// node.firstArc-tmpArcs: 	index into arcs array of last allocated out arc

	// copy to permanent arcs array, but saving tail instead of head
	if (verbose) {
		IBDEBUG("c initFast copy2");
	}
	a = arcs;
	x = nodes;
	taEnd = (tmpArcs+(arcEnd-arcs));
	for (ta=tmpArcs; ta != taEnd; ta++) {
		while (x->label <= (ta-tmpArcs)) x++;
		a->head = (x-1);
		a->rCap = ta->cap;
		a->rev = arcs + (ta->rev-tmpArcs);
		a++;
	}

	// tmpEdges:				overwritten
	// tmpArcs:					overwritten
	// arcs:					arcs array
	// node.label: 				index into arcs array of first out arc
	// node.firstArc-tmpArcs: 	index into arcs array of last allocated out arc
	// arc.head = tail of arc

	// swap the head and tail pointers and set isRevResidual
	if (verbose) {
		IBDEBUG("c initFast copy3");
	}
	for (a=arcs; a != arcEnd; a++) {
		if (a->rev <= a) continue;
		x = a->head;
		a->head = a->rev->head;
		a->rev->head = x;
		a->isRevResidual = (a->rev->rCap != 0);
		a->rev->isRevResidual = (a->rCap != 0);
	}

	// set firstArc pointers in nodes array
	if (verbose) {
		IBDEBUG("c initFast nodes");
	}
	for (x=nodes; x <= nodeEnd; x++) {
		x->firstArc = (arcs + x->label);
		if (x->excess == 0) {
			x->label = numNodes;
			continue;
		}
		if (x->excess > 0) {
			x->label = 1;
			activeS1.add(x);
		} else {
			x->label = -1;
			activeT1.add(x);
		}
	}

	// check consistency
	if (IBTEST) {
		IBDEBUG("c initFast test");
		for (x=nodes; x != nodeEnd; x++) {
			if ((x+1)->firstArc < x->firstArc) {
				fprintf(stderr, "INIT CONSISTENCY: arc pointers descending");
				exit(1);
			}
			for (a=x->firstArc; a !=(x+1)->firstArc; a++) {
				if (a->rev->head != x) {
					fprintf(stderr, "INIT CONSISTENCY: arc head pointer inconsistent");
					exit(1);
				}
				if (a->rev->rev != a) {
					fprintf(stderr, "INIT CONSISTENCY: arc reverse pointer inconsistent");
					exit(1);
				}
			}
		}
	}
}


void IBFSGraph::initGraphCompact()
{
	Arc *a, aTmp;
	Node *x, *y;

	// calculate start arc offsets every node
	for (x=(nodes+1); x != nodeEnd; x++) {
		x->label += (x-1)->label;
	}
	for (x=nodeEnd; x>nodes; x--) {
		x->label = (x-1)->label;
		x->firstArc = arcs + x->label;
	}
	nodes->label = 0;
	nodes->firstArc = arcs;

	// swap arcs
	for (x=nodes; x != nodeEnd; x++)
	{
		for (; x->firstArc != (arcs+((x+1)->label)); x->firstArc++)
		{
			for (y = x->firstArc->rev->head; y != x; y = x->firstArc->rev->head)
			{
				// get and advance last arc fwd in proper node
				a = y->firstArc;
				y->firstArc++;

				// prepare sister pointers
				if (a->rev == x->firstArc)
				{
					x->firstArc->rev = x->firstArc;
					a->rev = a;
				}
				else
				{
					a->rev->rev = x->firstArc;
					x->firstArc->rev->rev = a;
				}

				// swap
				aTmp = (*(x->firstArc));
				(*(x->firstArc)) = (*a);
				(*a) = aTmp;
			}
		}
	}

	// reset first arc pointers
	// and sister_rCap
	for (x=nodes; x <= nodeEnd; x++)
	{
		if (x != nodeEnd) {
			x->firstArc = arcs + x->label;
			x->label = 0;
		}
		if (x != nodes) {
			for (a=(x-1)->firstArc; a != x->firstArc; a++)
			{
				if (a->rev->rCap == 0) {
					a->isRevResidual = 0;
				} else {
					a->isRevResidual = 1;
				}
			}
		}
	}
}


template <bool sTree>
void IBFSGraph::augmentTree(Node *x, EdgeCap bottleneck)
{
	Node *y;
	Arc *a;

	for (; ; x=a->head)
	{
		if (x->excess) break;
		a = x->parent;
		if (sTree) {
			a->rCap += bottleneck;
			a->rev->isRevResidual = 1;
			a->rev->rCap -= bottleneck;
		} else {
			a->rev->rCap += bottleneck;
			a->isRevResidual = 1;
			a->rCap -= bottleneck;
		}

		// saturated?
		if ((sTree ? (a->rev->rCap) : (a->rCap)) == 0)
		{
			if (sTree) a->isRevResidual = 0;
			else a->rev->isRevResidual = 0;
			y=x->parent->head->firstSon;
			if (y == x) {
				x->parent->head->firstSon = x->nextPtr;
			} else {
				for (; y->nextPtr != x; y = y->nextPtr);
				y->nextPtr = x->nextPtr;
			}
			ADD_ORPHAN_FRONT(x);
		}
	}
	x->excess += (sTree ? -bottleneck : bottleneck);
	if (x->excess == 0) {
		ADD_ORPHAN_FRONT(x);
	}
}


void IBFSGraph::augment(Arc *bridge)
{
	Node *x;
	Arc *a;
	EdgeCap bottleneck;
	Real pushesBefore;

	// stats
	if (IBSTATS) pushesBefore=stats.getPushes();
	stats.incAugs();
	stats.incPushes();

	// bottleneck in S
	bottleneck = bridge->rCap;
	for (x=bridge->rev->head; ; x=a->head)
	{
		stats.incPushes();
		if (x->excess) break;
		a = x->parent;
		if (bottleneck > a->rev->rCap) {
			bottleneck = a->rev->rCap;
		}
	}
	if (bottleneck > x->excess) {
		bottleneck = x->excess;
	}

	// bottleneck in T
	for (x=bridge->head; ; x=a->head)
	{
		stats.incPushes();
		if (x->excess) break;
		a = x->parent;
		if (bottleneck > a->rCap) {
			bottleneck = a->rCap;
		}
	}
	if (bottleneck > (-x->excess)) {
		bottleneck = (-x->excess);
	}

	// stats
	if (IBSTATS) {
		Real augLen = stats.getPushes() - pushesBefore;
		stats.addAugLen(augLen);
	}

	// augment connecting arc
	bridge->rev->rCap += bottleneck;
	bridge->isRevResidual = 1;
	bridge->rCap -= bottleneck;
	if (bridge->rCap == 0) {
		bridge->rev->isRevResidual = 0;
	}

	// augment T
	augTimestamp++;
	augmentTree<false>(bridge->head, bottleneck);
	adoption<false>();

	// augment S
	augTimestamp++;
	augmentTree<true>(bridge->rev->head, bottleneck);
	adoption<true>();

	flow += bottleneck;
}


template <bool sTree>
void IBFSGraph::adoption()
{
	Node *x, *y, *z;
	Arc *a, *aEnd;
	bool threePass;
	int minLabel, numOrphans, numOrphansUniq;

	threePass=false;
	numOrphans=0;
	numOrphansUniq=0;
	while (orphanFirst != IB_ORPHANS_END)
	{
		x = orphanFirst;
		orphanFirst = x->nextPtr;
		//x->nextOrphan = NULL;
		testNode(x);
		stats.incOrphans();
		numOrphans++;
		if (x->lastAugTimestamp != augTimestamp) {
			x->lastAugTimestamp = augTimestamp;
			if (sTree) uniqOrphansS++;
			else uniqOrphansT++;
			numOrphansUniq++;
		}
		if (numOrphans >= 3*numOrphansUniq) {
			// switch to 3pass
			threePass = true;
		}

		// check for same level connection
		if (x->isParentCurr) {
			a = x->parent;
		} else {
			a = x->firstArc;
			x->isParentCurr = 1;
		}
		x->parent = NULL;
		aEnd = (x+1)->firstArc;
		if (x->label != (sTree ? 1 : -1))
		{
			minLabel = x->label - (sTree ? 1 : -1);
			for (; a != aEnd; a++)
			{
				stats.incOrphanArcs1();
				y = a->head;
				if ((sTree ? a->isRevResidual : a->rCap) != 0 &&
					y->label == minLabel)
				{
					x->parent = a;
					x->nextPtr = y->firstSon;
					y->firstSon = x;
					break;
				}
			}
		}
		if (x->parent != NULL) continue;

		// give up on same level - relabel it!
		// (1) create orphan sons
		for (y=x->firstSon; y != NULL; y=z)
		{
			stats.incOrphanArcs3();
			z=y->nextPtr;
			ADD_ORPHAN_BACK(y);
		}
		x->firstSon = NULL;

		// on the top level there is no need to relabel
		if (x->label == (sTree ? topLevelS : -topLevelT)) {
			x->label = numNodes;
			continue;
		}

		// 3pass relabeling: move to buckets structure
		if (threePass) {
			x->label += (sTree ? 1 : -1);
			orphanBuckets.add<sTree>(x);
			continue;
		}

		// (2) relabel: find the lowest level parent
		minLabel = (sTree ? topLevelS : -topLevelT);
		if (x->label != minLabel) for (a=x->firstArc; a != aEnd; a++)
		{
			stats.incOrphanArcs2();
			y = a->head;
			if ((sTree ? a->isRevResidual : a->rCap) &&
				// y->label != numNodes ---> holds implicitly
				(sTree ? (y->label > 0) : (y->label < 0)) &&
				(sTree ? (y->label < minLabel) : (y->label > minLabel)))
			{
				minLabel = y->label;
				x->parent = a;
				if (minLabel == x->label) break;
			}
		}

		// (3) relabel onto new parent
		if (x->parent != NULL) {
			x->label = minLabel + (sTree ? 1 : -1);
			x->nextPtr = x->parent->head->firstSon;
			x->parent->head->firstSon = x;
			// add to active list of the next growth phase
			if (sTree) {
				if (x->label == topLevelS) activeS1.add(x);
			} else {
				if (x->label == -topLevelT) activeT1.add(x);
			}
		} else {
			x->label = numNodes;
		}
	}

	if (threePass) {
		adoption3Pass<sTree>();
	}
}

template <bool sTree>
void IBFSGraph::adoption3Pass()
{
	Arc *a, *aEnd;
	Node *x, *y;
	int minLabel, destLabel;

	for (int level=2; level <= orphanBuckets.maxBucket; level++)
	{
		while ((x = orphanBuckets.popFront(level)) != NULL)
		{
			testNode(x);
			aEnd = (x+1)->firstArc;

			// pass 2: find lowest level parent
			if (x->parent == NULL) {
				minLabel = (sTree ? topLevelS : -topLevelT);
				destLabel = x->label - (sTree ? 1 : -1);
				for (a=x->firstArc; a != aEnd; a++) {
					y = a->head;
					if ((sTree ? a->isRevResidual : a->rCap) &&
						(y->excess || y->parent != NULL) &&
						//!y->isOrphan() &&
						(sTree ? (y->label > 0) : (y->label < 0)) &&
						(sTree ? (y->label < minLabel) : (y->label > minLabel)))
					{
						x->parent = a;
						if ((minLabel = y->label) == destLabel) break;
					}
				}
				if (x->parent == NULL) {
					x->label = numNodes;
					continue;
				}
				x->label = minLabel + (sTree ? 1 : -1);
				if (x->label != (sTree ? level : -level)) {
					orphanBuckets.add<sTree>(x);
					continue;
				}
			}

			// pass 3: lower potential sons and/or find first parent
			if (x->label != (sTree ? topLevelS : -topLevelT))
			{
				minLabel = x->label + (sTree ? 1 : -1);
				for (a=x->firstArc; a != aEnd; a++) {
					y = a->head;

					// lower potential sons
					if ((sTree ? a->rCap : a->isRevResidual) &&
						((!sTree && y->label == numNodes) ||
						// the above implicitly holds by condition below when sTree=true
						(sTree ? (minLabel < y->label) : (minLabel > y->label))))
					{
						if (y->label != numNodes) orphanBuckets.remove<sTree>(y);
						y->label = minLabel;
						y->parent = a->rev;
						orphanBuckets.add<sTree>(y);
					}
				}
			}

			// relabel onto new parent
			x->nextPtr = x->parent->head->firstSon;
			x->parent->head->firstSon = x;
			x->isParentCurr = 0;
			// add to active list of the next growth phase
			if (sTree) {
				if (x->label == topLevelS) activeS1.add(x);
			} else {
				if (x->label == -topLevelT) activeT1.add(x);
			}
		}
	}

	orphanBuckets.maxBucket = 0;
}


template <bool dirS>
void IBFSGraph::growth()
{
	Node *x, *y;
	Arc *a, *aEnd;

	for (Node **active=active0.list; active != (active0.list + active0.len); active++)
	{
		// get active node
		x = (*active);
		testNode(x);

		// node no longer at level
		if (x->label != (dirS ? (topLevelS-1): -(topLevelT-1))) {
			continue;
		}

		// grow or augment
		if (dirS) stats.incGrowthS();
		else stats.incGrowthT();
		aEnd = (x+1)->firstArc;
		for (a=x->firstArc; a != aEnd; a++)
		{
			stats.incGrowthArcs();
			if ((dirS ? a->rCap : a->isRevResidual) == 0) continue;
			y = a->head;
			if (y->label == numNodes)
			{
				// grow node
				testNode(y);
				y->isParentCurr = 0;
				y->label = x->label + (dirS ? 1 : -1);
				y->parent = a->rev;
				y->nextPtr = x->firstSon;
				x->firstSon = y;
				if (dirS) activeS1.add(y);
				else activeT1.add(y);
			}
			else if (dirS ? (y->label < 0) : (y->label > 0))
			{
				// augment
				augment(dirS ? a : (a->rev));
				if (x->label != (dirS ? (topLevelS-1) : -(topLevelT-1))) {
					break;
				}
				if (dirS ? (a->rCap) : (a->isRevResidual)) a--;
			}
		}
	}
	active0.clear();
}


void IBFSGraph::testTree()
{
	Node *x;
	Arc *a;

	for (x=nodes; x != nodeEnd; x++) {
		if (x->label != numNodes && (x->label > topLevelS || x->label < -topLevelT)) {
			IBDEBUG("ILLEGAL LABEL!");
			testExit();
		}
		if (x->parent == NULL) continue;
		bool sTree = (x->label > 0);
		if (x->label == (sTree ? topLevelS : -topLevelT)) {
			continue;
		}
		for (a=x->firstArc; a != (x+1)->firstArc; a++) {
			if (x->isParentCurr &&
					(sTree ? a->isRevResidual : a->rCap) &&
					(sTree ? (a->head->label > 0) : (a->head->label < 0)) &&
					a->head->label == (sTree ? (x->label-1) : (x->label+1)) &&
					a < x->parent) {
				IBDEBUG("ILLEGAL CURRENT ARC!");
				testExit();
			}
			if (!(sTree ? a->rCap : a->isRevResidual)) continue;
			if (a->head->parent == NULL) {
				IBDEBUG("CROSS OUT NODE!");
				testExit();
			}
			if (sTree ? (a->head->label < 0) : (a->head->label > 0)) {
				IBDEBUG("CROSS NODE!");
				testExit();
			}
			if (sTree ? (a->head->label > (x->label+1)) : (a->head->label < (x->label-1))) {
				IBDEBUG("EXTENDED ARC!");
				testExit();
			}
		}
	}
}

EdgeCap IBFSGraph::computeMaxFlow()
{
	// init
	orphanFirst = IB_ORPHANS_END;
	topLevelS = topLevelT = 1;
	bool dirS = true;
	ActiveList::swapLists(&active0, &activeS1);

	//
	// IBFS
	//
	while (true)
	{
		// BFS level
		if (dirS) topLevelS++;
		else topLevelT++;
		if (dirS) growth<true>();
		else growth<false>();
		if (IBTEST) {
			testTree();
			fprintf(stdout, "dirS=%d aug=%d   S %d / T %d\n", dirS, augTimestamp, uniqOrphansS, uniqOrphansT);
			fflush(stdout);
		}

		// switch to next level
		if (activeS1.len == 0 || activeT1.len == 0) {
			break;
		}
		if ((!IB_ALTERNATE_SMART && dirS) ||
			(IB_ALTERNATE_SMART && uniqOrphansT == uniqOrphansS && dirS) ||
			(IB_ALTERNATE_SMART && uniqOrphansT < uniqOrphansS)) {
			// grow T
			ActiveList::swapLists(&active0, &activeT1);
			dirS=false;
		} else {
			// grow S
			ActiveList::swapLists(&active0, &activeS1);
			dirS=true;
		}
	}
	
	return flow;
}


#if IBIO>0
bool IBFSGraph::readFromFile(char *filename)
{
	return readFromFile(filename, false);
}
bool IBFSGraph::readFromFileCompile(char *filename)
{
	return readFromFile(filename, true);
}
bool IBFSGraph::readFromFile(char *filename, bool checkCompile)
{
	const int MAX_LINE_LEN = 100;
	char line[MAX_LINE_LEN];
	int declaredNumOfNodes, declaredNumOfEdges, nodeId1, nodeId2;
	int currentNumOfEdges = 0;
	char c, c1, c2, c3;
	EdgeCap capacity, capacity2;
	int numLines=0;
	// only for compile mode
	const int bufferSize = sizeof(char) + sizeof(EdgeCap)*4;
	char buffer[bufferSize];

	char *filenameCompiled = new char[strlen(filename) + strlen(".compiled") + 1];
	strcpy(filenameCompiled, filename);
	strcat(filenameCompiled, ".compiled");

	FILE *pFile;
	FILE *pFileCompiled = NULL;
	if (checkCompile) {
		if ((pFileCompiled = fopen(filenameCompiled, "rb")) != NULL) {
			delete[] filenameCompiled;
			return readCompiled(pFileCompiled);
		}
		fclose(pFileCompiled);
	}
	if ((pFile = fopen(filename, "r")) == NULL) {
		fprintf(stdout, "Could not open file %s\n", filename);
		delete[] filenameCompiled;
		return false;
	}
	if (checkCompile && (pFileCompiled = fopen(filenameCompiled, "wb")) == NULL) {
		fprintf(stdout, "Could not open file %s\n", filenameCompiled);
		delete[] filenameCompiled;
		fclose(pFile);
		return false;
	}
	delete[] filenameCompiled;

	// read from file into temporary structure
	while (fgets(line, MAX_LINE_LEN, pFile) != NULL)
	{
		numLines++;
		switch (line[0])
	    {
			case 'c':
			case '\n':
			case '\0':
			default:
				break;
			case 'p':
				sscanf(line, "%c %c%c%c", &c, &c1, &c2, &c3);
				if (c1=='m' && c2=='a' && c3=='x') {
					sscanf(line, "%c %c%c%c %d %d", &c, &c1, &c2, &c3, &declaredNumOfNodes, &declaredNumOfEdges);
				} else {
					sscanf(line, "%c %d %d", &c, &declaredNumOfNodes, &declaredNumOfEdges);
				}
				initSize(declaredNumOfNodes, declaredNumOfEdges);
				if (checkCompile) {
					fwrite(&declaredNumOfNodes, sizeof(int), 1, pFileCompiled);
					fwrite(&declaredNumOfEdges, sizeof(int), 1, pFileCompiled);
				}
				break;

			case 'n':
				sscanf(line, "%c %d %d %d ", &c, &nodeId1, &capacity, &capacity2);
				if (capacity != 0 || capacity2 != 0) {
					addNode(nodeId1, capacity, capacity2);
					if (checkCompile) {
						buffer[0] = 'n';
						memcpy(buffer+sizeof(char), &nodeId1, sizeof(int));
						memcpy(buffer+sizeof(char)+sizeof(int), &nodeId1, sizeof(int));
						memcpy(buffer+sizeof(char)+sizeof(int)+sizeof(int), &capacity, sizeof(int));
						memcpy(buffer+sizeof(char)+sizeof(int)+sizeof(int)+sizeof(int), &capacity2, sizeof(int));
						fwrite(&buffer, 1, bufferSize, pFileCompiled);
					}
				}
				break;

			case 'a':
				sscanf(line, "%c %d %d %d %d", &c,
					&nodeId1, &nodeId2, &capacity, &capacity2);
				if (nodeId1 < 0 ||
					nodeId1 >= declaredNumOfNodes ||
					nodeId2 < 0 ||
					nodeId2 >= declaredNumOfNodes)
				{
					fprintf(stdout, "inconsistent node index (Line %d)\n", numLines);
					return false;
				}
				if (currentNumOfEdges >= declaredNumOfEdges)
				{
					fprintf(stdout, "inconsistent number of edges (Line %d)\n", numLines);
					return false;
				}
				addEdge(nodeId1, nodeId2, capacity, capacity2);
				currentNumOfEdges++;
				if (checkCompile) {
					buffer[0] = 'a';
					memcpy(buffer+sizeof(char), &nodeId1, sizeof(int));
					memcpy(buffer+sizeof(char)+sizeof(int), &nodeId2, sizeof(int));
					memcpy(buffer+sizeof(char)+sizeof(int)+sizeof(int), &capacity, sizeof(int));
					memcpy(buffer+sizeof(char)+sizeof(int)+sizeof(int)+sizeof(int), &capacity2, sizeof(int));
					fwrite(&buffer, 1, bufferSize, pFileCompiled);
				}
				break;
		}
	}

	fclose(pFile);
	if (checkCompile) {
		buffer[0] = 'x';
		fwrite(&buffer, 1, bufferSize, pFileCompiled);
	}
	if (currentNumOfEdges != declaredNumOfEdges) {
		fprintf(stdout, "inconsistent number of edges: differs from declared %d != %d\n",
				currentNumOfEdges, declaredNumOfEdges);
		return false;
	}
	return true;
}


bool IBFSGraph::readCompiled(FILE *pFile)
{
	int declaredNumOfNodes, declaredNumOfEdges, nodeId1, nodeId2;
	EdgeCap capacity, capacity2;
	const int bufferSize = sizeof(char)+sizeof(EdgeCap)*4;
	char buffer[bufferSize];

	// read from file into htemporary structure
	fprintf(stdout, "c reading compiled file\n");
	if (fread(&declaredNumOfNodes, sizeof(int), 1, (pFile)) < 1 ||
			fread(&declaredNumOfEdges, sizeof(int), 1, (pFile)) < 1) {
		fprintf(stdout, "ERROR while reading compiled num nodes/edges, EOF=%d\n", feof(pFile));
		fclose(pFile);
		return false;
	}
	initSize(declaredNumOfNodes, declaredNumOfEdges);
	for (int line=0; !feof(pFile); line++) {
		if (fread(&buffer, 1, bufferSize, pFile) < bufferSize) {
			fprintf(stdout, "ERROR while reading compiled line %d, EOF=%d\n", line, feof(pFile));
			fclose(pFile);
			return false;
		}
		memcpy(&nodeId1,   buffer+sizeof(char), sizeof(int));
		memcpy(&nodeId2,   buffer+sizeof(char)+sizeof(int), sizeof(int));
		memcpy(&capacity,  buffer+sizeof(char)+sizeof(int)+sizeof(int), sizeof(int));
		memcpy(&capacity2, buffer+sizeof(char)+sizeof(int)+sizeof(int)+sizeof(int), sizeof(int));
		if (buffer[0] == 'n') {
			if (capacity != 0 || capacity2 != 0) {
				addNode(nodeId1, capacity, capacity2);
			}
		} else if (buffer[0] == 'a') {
			if (nodeId1 < 0 ||
				nodeId1 >= declaredNumOfNodes ||
				nodeId2 < 0 ||
				nodeId2 >= declaredNumOfNodes)
			{
				fprintf(stdout, "inconsistent node index in compiled file %d,%d line %d\n", nodeId1, nodeId2, line);
				return false;
			}
			addEdge(nodeId1, nodeId2, capacity, capacity2);
		} else if (buffer[0] == 'x') {
			break;
		}
	}
	fclose(pFile);
	return true;
}
#endif