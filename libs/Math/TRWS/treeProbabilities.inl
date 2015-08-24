template <class T> void MRFEnergy<T>::SetMonotonicTrees()
{
	Node* i;
	MRFEdge* e;

	if (!m_isEnergyConstructionCompleted)
	{
		CompleteGraphConstruction();
	}

	for (i=m_nodeFirst; i; i=i->m_next)
	{
		REAL mu;

		int nForward = 0, nBackward = 0;
		for (e=i->m_firstForward; e; e=e->m_nextForward)
		{
			nForward ++;
		}
		for (e=i->m_firstBackward; e; e=e->m_nextBackward)
		{
			nBackward ++;
		}
		int ni = (nForward > nBackward) ? nForward : nBackward;

		mu = (REAL)1 / ni;
		for (e=i->m_firstBackward; e; e=e->m_nextBackward)
		{
			e->m_gammaBackward = mu;
		}
		for (e=i->m_firstForward; e; e=e->m_nextForward)
		{
			e->m_gammaForward = mu;
		}
	}
}
