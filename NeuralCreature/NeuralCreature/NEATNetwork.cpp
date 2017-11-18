#include "stdafx.h"
#include "NEATNetwork.h"

int l = 0;
bool moreThanOREqualById(NEATGene& gene1, NEATGene& gene2) { return (gene1.getOutId() > gene2.getOutId()); }
bool moreThanOrEqualByInnovation(NEATGene& gene1, NEATGene& gene2) { return (gene1.getInnovationNumber() > gene2.getInnovationNumber()); }
bool moreThanByIdNetwork(NEATNeuron& neuron1, NEATNeuron& neuron2) { return (neuron1.getId() > neuron2.getId()); }

void NEATNetwork::printFirstWeight()
{
	std::cout << m_genes[16].getWeight() << "\n";
	//for (int i = 0; i < m_genes.size(); i++) {
	//	std::cout << m_genes[i].getWeight() << " ";
	//}
	//std::cout << "\n";
}

NEATNetwork::NEATNetwork()
{
}

void NEATNetwork::generateNeuralNetworkFromGene()
{

	m_usedHiddenNeuronIndex = 0;
	for (int i = 0; i < m_genes.size(); i++)
	{
		int inNode = m_genes[i].getInId();
		int outNode = m_genes[i].getOutId();

		if (m_usedHiddenNeuronIndex < inNode)
			m_usedHiddenNeuronIndex = inNode;

		if (m_usedHiddenNeuronIndex < outNode)
			m_usedHiddenNeuronIndex = outNode;
	}

	m_usedHiddenNeuronIndex = m_usedHiddenNeuronIndex + 1; //incremented as per LSES algorithm 

	m_network.clear();
	for (int i = 0; i < m_usedHiddenNeuronIndex; i++)
	{
		NEATNeuron neuron(i, 0.f);
		m_network.push_back(neuron);
	}

	//m_network.Sort((x, y) = > x.id.CompareTo(y.id));
	//m_genes.Sort((x, y) = > x.GetOutID().CompareTo(y.GetOutID()));
	std::sort(m_network.begin(), m_network.end(), moreThanByIdNetwork);
	std::sort(m_genes.begin(), m_genes.end(), moreThanOREqualById);

	for (int i = 0; i < m_genes.size(); i++)
	{
		NEATGene gene = m_genes[i];

		if (gene.getGeneState())
		{
			m_network[gene.getOutId()].inc.push_back(gene);
		}
	}

	//networkArray = network.ToArray();

	for (int i = 0; i < m_network.size(); i++)
	{
		std::sort(m_network[i].inc.begin(), m_network[i].inc.end(), moreThanOREqualById);
		//m_network[i].incommingArray = networkArray[i].incomming.ToArray();
	}

	std::sort(m_genes.begin(), m_genes.end(), moreThanOrEqualByInnovation);
}

std::vector<double> NEATNetwork::forward(std::vector<double>& inputVals)
{
	for (int i = 0; i < inputVals.size(); i++)
	{
		m_network[i].setOutputValue(inputVals[i]);

	}
	//float[] output = new float[numberOfOutputs];
	//float[] tempValues = new float[networkArray.Length];
	std::vector<double> tempValues;

	for (int i = 0; i < m_network.size(); i++)
		tempValues.push_back(m_network[i].getOutputValue());

	//Set bias to 1.f
	m_network[m_numInputs - 1].setOutputValue(1.f);


	for (int i = 0; i < m_network.size(); i++)
	{
		double sum = 0;
		NEATNeuron& neuron = m_network[i];
		std::vector<NEATGene> incomingGenes = neuron.inc;

		if (incomingGenes.size() > 0)
		{
			for (int j = 0; j < incomingGenes.size(); j++)
			{
				if (incomingGenes[j].getGeneState())
				{
					sum = sum + (incomingGenes[j].getWeight() * tempValues[incomingGenes[j].getInId()]);
				}
			}
			neuron.setOutputValue(tanh(sum));
		}
	}


	std::vector<double> outputs;
	for (int i = 0; i < outputs.size(); i++)
	{
		outputs[i] = m_network[i + m_numInputs].getOutputValue();

	}

	return outputs;
}

void NEATNetwork::getResults(std::vector<double>& resultsVals)
{
	resultsVals.clear();
	for (int i = 0; i < m_numOutputs; i++)
	{
		resultsVals.push_back(m_network[i + m_numInputs].getOutputValue());
	}
}

NEATNetwork::NEATNetwork(NEATNetwork & network)
{
	m_controller = network.m_controller; //shallow copy consultor
	m_numInputs = network.m_numInputs; //copy number of inputs
	m_numOutputs = network.m_numOutputs; //copy number of outputs

	//CopyNodes(copy.nodeList); //deep copy node list
	//CopyGenes(copy.geneList); //deep copy gene list
	m_genes = network.m_genes;
	m_nodes = network.m_nodes;
	m_netID = network.m_netID;
	m_fitness = network.m_fitness;
	m_timeAlive = network.m_timeAlive;
	m_network = network.m_network;
	generateNeuralNetworkFromGene(); // m_network copy not working fix
}


NEATNetwork::NEATNetwork(NEATController controller, int netId, int numInputs, int numOutputs)
{
	m_controller = controller; 
	m_netID = netId; 
	m_numInputs = numInputs;
	m_numOutputs = numOutputs;

	m_fitness = 0.f; 
	m_timeAlive= 0.f; 
	createNodes();
	createGenes();

	generateNeuralNetworkFromGene();
}

void NEATNetwork::createNodes()
{
	//m_nodes
	for (int i = 0; i < m_numInputs; i++) { //run through number of input perceptrons

		NEATNode node(i, (i == (m_numInputs - 1)) ? INPUT_BIAS_NEURON : INPUT_NEURON); //Make node with index i as node ID and bias or input type
	
		m_nodes.push_back(node); //add node to the node list
	}

	for (int i = m_numInputs; i < m_numInputs + m_numOutputs; i++) {  //run through number of output perceptrons
		NEATNode node(i, OUTPUT_NEURON); //Make output neurons with index i as id
		m_nodes.push_back(node); //add node to the node list
	}
}

void NEATNetwork::createGenes()
{

	for (int i = 0; i < m_numInputs; i++) { //run through number of inputs
		for (int j = m_numInputs; j < m_numInputs + m_numOutputs; j++) { //run through number of outputs
			int inno = m_controller.checkGeneExistence(i, j);  //check if gene exists in controller
			double randWeight = ((double)rand() / (RAND_MAX)) * 2 - 1;
			NEATGene gene(inno, i, j, randWeight, true);
			insertNewGene(gene); //insert gene to correct location in gene list
		}
	}
}

double NEATNetwork::getFitness()
{
	return m_fitness;
}

double NEATNetwork::getTimeAlive()
{
	return m_timeAlive;
}

void NEATNetwork::setId(int id)
{
	m_netID = id;
}

void NEATNetwork::addToFitness(double fitness)
{
	m_fitness += fitness;
}

void NEATNetwork::setTimeAlive(double timeAlive)
{
	m_timeAlive = timeAlive;
}

void NEATNetwork::addToTimeAlive(double timeAlive)
{
	m_timeAlive += timeAlive;
}

int NEATNetwork::getNodeCount()
{
	return m_nodes.size();
}

int NEATNetwork::getNumInputNodes()
{
	return m_numInputs;
}

int NEATNetwork::getNumOutputNodes()
{
	return m_numOutputs;
}

NEATController& NEATNetwork::getController()
{
	return m_controller;
}

void NEATNetwork::setInputValues(std::vector<double> inputs)
{
	for (int i = 0; i < m_numInputs; i++) { //run through number of inputs
		if (m_nodes[i].getNeuronType() == INPUT_NEURON) { //only if this is a input node
			m_nodes[i].setOutputValue(inputs[i]); //change value of node to given value at index i
		}
		else { //if this is not an input type node
			break;
		}
	}
}

std::vector<double> NEATNetwork::getOutputValues()
{
	std::vector<double> outputValues;
	for (int i = 0; i < m_numOutputs; i++) { //run through number of outputs
		outputValues.push_back(m_nodes[i + m_numInputs].getOutputValue());//set output nodes value
	}

	return outputValues; //return output nodes value vector
}

void NEATNetwork::mutate()
{
	double randomNumber = ((double)rand() / (RAND_MAX)); //random number between 0 and 1
	double chance = 0.25; //25% chance of mutation
	//std::cout << "randomNumber: " << randomNumber << "\n";
	if (randomNumber <= chance) { //random number is below chance
		std::cout << "addConnection\n";
		addConnection(); //add connection between 2 nodes
	}
	else if (randomNumber <= (chance * 2)) {//random number is below chance*2
		std::cout << "addNode\n";
		addNode(); //add a new node bettwen an existing connection
	}

	mutateWeight(); //mutate weight
	generateNeuralNetworkFromGene();

}

void NEATNetwork::addConnection()
{
	int tempNode1, tempNode2, innovationNumber; //random node ID's and innovation number
	//int totalAttemptsAllowed = (int)Mathf.Pow(nodeList.Count, 2); //total attempts allowed to find two unconnected nodes
	int totalAttemptsAllowed = m_nodes.size() * m_nodes.size();
	bool found = false; //used to check if a connection is found

	while (totalAttemptsAllowed > 0 && found == false) { //if connection is found and greater than 0 attempts left
		tempNode1 = rand() % m_nodes.size();//pick a random node
		//randomNodeID2 = Random.Range(numberOfInputs, nodeList.Count); //pick a random node that is not the input
		tempNode2 = rand() % m_nodes.size() + m_numInputs;
		if (!connectionExists(tempNode1, tempNode2)) { //if connection does not exist with random node 1 as in node and random node 2 and out node
			innovationNumber = m_controller.checkGeneExistence(tempNode1, tempNode2); //get the new innovation number
			NEATGene gene(innovationNumber, tempNode1, tempNode2, 1.f, true);//create gene which is enabled and 1 as default weight
			insertNewGene(gene); //add gene to the gene list

			found = true; //connection made
		}
		else if (m_nodes[tempNode1].getNeuronType() > 1 && !connectionExists(tempNode2, tempNode1)) { //if random node 1 isn't input type and connection does not exist with random node 2 as in node and random node 1 and out node
			innovationNumber = m_controller.checkGeneExistence(tempNode2, tempNode1); //get the new innovation number
			NEATGene gene(innovationNumber, tempNode1, tempNode2, 1.f, true);//create gene which is enabled and 1 as default weight
			insertNewGene(gene); //add gene to the gene list

			found = true; //connection made
		}

		if (tempNode1 == tempNode2) //both random nodes are equal
			totalAttemptsAllowed--; //only one attemp removed becuase only 1 connection can be made
		else //both nodes are different
			totalAttemptsAllowed -= 2; //two connections can be made
	}

	if (found == false) { //if not found and attempts ran out
		addNode(); //
	}
}

void NEATNetwork::addNode()
{
	int firstID, secondID, thirdID, inno; //first ID is old connections in node, third ID is old connections out node, second ID is the new node, and new innovation number for the connections
										  //int randomGeneIndex = Random.Range(0, geneList.Count); //find a random gene

	float oldWeight; //weight from the old gene

					 //NEATGene oldGene = geneList[randomGeneIndex]; //get old gene

	NEATGene oldGene; //find a random old gene
	bool found = false; //used to check if old gene is found
	while (!found) { //run till found
		int randomGeneIndex = rand() & (m_genes.size()-1); //pick random gene
		oldGene = m_genes[randomGeneIndex]; //get gene at random index
		if (oldGene.getGeneState() == true) { //if gene is active
			found = true; //found
		}
	}
	oldGene.setGeneState(false); //disable this gene
	firstID = oldGene.getInId(); //get in node ID
	thirdID = oldGene.getOutId(); //get out node ID
	oldWeight = oldGene.getWeight(); //get old weight

	NEATNode newNode(m_genes.size(), HIDDEN_NEURON);//create new hidden node
	m_nodes.push_back(newNode); //add new node to the node list
	secondID = newNode.getNeuronId(); //get new node's ID

	inno = m_controller.checkGeneExistence(firstID, secondID); //get new innovation number for new gene
	NEATGene newGene1(inno, firstID, secondID, 1.f, true);//create new gene

	inno = m_controller.checkGeneExistence(secondID, thirdID); //get new innovation number for new gene
	NEATGene newGene2(inno, secondID, thirdID, oldWeight, true);//create new gene
	
	//add genes to gene list
	insertNewGene(newGene1);
	insertNewGene(newGene2);
}

void NEATNetwork::mutateWeight()
{
	int numberOfGenes = m_genes.size(); //number of genes

	for (int i = 0; i < numberOfGenes; i++) { //run through all genes
		NEATGene& gene = m_genes[i]; // get gene at index i
		double weight = 0;

		int randomNumber = rand() % 3 + 1;

		if (randomNumber <= 1) { //if 1
								 //flip sign of weight
			weight = gene.getWeight();
			weight *= -1.f;
			gene.setWeight(weight);
		}
		else if (randomNumber <= 2) { //if 2
									  //pick random weight between -1 and 1
			weight = ((double)rand() / (RAND_MAX)) * 2 - 1;
			gene.setWeight(weight);
		}
		else if (randomNumber <= 3) { //if 3
									  //randomly increase/decrease by 0% to 100%
			float factor = ((double)rand() / (RAND_MAX)) * 2 - 1;
			weight = gene.getWeight() * factor;
			gene.setWeight(weight);
		}
		else if (randomNumber <= 4) { //if 5
									  //flip activation state for gene
									  //gene.SetGeneState(!gene.GetGeneState());
		}
	}

}

bool NEATNetwork::connectionExists(int inId, int outId)
{
	int numberOfGenes = m_genes.size(); //number of genes

	for (int i = 0; i < numberOfGenes; i++) { //run through gene list
		int nodeInID = m_genes[i].getInId(); //get in node
		int nodeOutID = m_genes[i].getOutId(); //get out node

		if (nodeInID == inId && nodeOutID == outId) { //check if nodes match given parameters
			return true; //return true
		}
	}

	return false; //return false if no match
}

void NEATNetwork::insertNewGene(NEATGene gene)
{
	int inno = gene.getInnovationNumber(); //get innovation number
	int insertIndex = findInnovationInsertIndex(inno); //get insert index

	if (insertIndex == m_genes.size()) { //if insert index is equal to the size of the genome
		m_genes.push_back(gene); //add gene 
	}
	else { //otherwise
		m_genes.insert(m_genes.begin() + insertIndex, gene); //add gene to the given insert index location
	}
}

int NEATNetwork::findInnovationInsertIndex(int innovationNumber)
{
	int numberOfGenes = m_genes.size(); //number of genes
	int startIndex = 0; //start index
	int endIndex = numberOfGenes - 1; //end index

	if (numberOfGenes == 0) { //if there are no genes
		return 0; //first location to insert
	} else if (numberOfGenes == 1) { //if there is only 1 gene
		if (innovationNumber > m_genes[0].getInnovationNumber()) { //if innovation is greater than the girst gene's innovation
			return 1; //insert into second location
		}
		else {
			return 0; //insert into first location 
		}
	}
	int insertIndex = 0;
	while (true) { //run till found
		int middleIndex = (endIndex + startIndex) / 2; //find middle index (middle of start and end)
		int middleInno = m_genes[middleIndex].getInnovationNumber(); //get middle index's innovation number

		if (endIndex - startIndex == 1) { //if there is only 1 index between start and end index (base case on recursion)
			int endInno = m_genes[endIndex].getInnovationNumber(); //get end inde's innovation
			int startInno = m_genes[startIndex].getInnovationNumber(); //get start index's innovation

			if (innovationNumber < startInno) { //innovation is less than start innovation
				return startIndex; //return start index
			}
			else if (innovationNumber > endInno) { //innovation is greater than end innovation
				return endIndex + 1; //return end index + 1
			}
			else {
				return endIndex; //otherwise right in end index
			}
		}
		else if (innovationNumber > middleInno) { //innovation is greater than middle innovation
			startIndex = middleIndex; //new start index will be the middle
		}
		else { //innovation is less than middle innovation
			endIndex = middleIndex; //new end index is middle index
		}
	}
}

NEATNetwork NEATNetwork::crossover(NEATNetwork parent1, NEATNetwork parent2)
{
	//NEATNetwork child; //child to create

	//Hashtable geneHash = new Hashtable(); //hash table to be used to compared genes from the two parents

	//std::vector<NEATGene> childGeneList; //new gene child gene list to be created
	//std::vector<NEATNode> childNodeList; //new child node list to be created

	//std::vector<NEATGene> geneList1 = parent1.m_genes; //get gene list of the parent 1
	//std::vector<NEATGene> geneList2 = parent2.m_genes; //get gene list of parent 2

	//NEATController controller = parent1.getController(); //get consultor (consultor is the same for all neural network as it's just a pointer location)

	//int numberOfGenes1 = geneList1.size(); //get number of genes in parent 1
	//int numberOfGenes2 = geneList2.size(); //get number of genes in parent 2
	//int numberOfInputs = parent1.getNumInputNodes(); //number of inputs (same for both parents)
	//int numberOfOutputs = parent1.getNumOutputNodes(); //number of outputs (same for both parents)

	//if (parent1.getNodeCount() > parent2.getNodeCount()) { //if parents 1 has more nodes than parent 2
	//	childNodeList = parent1.m_nodes; //copy parent 1's node list
	//}
	//else { //otherwise parent 2 has euqal and more nodes than parent 1
	//	childNodeList = parent2.m_nodes; //copy parent 2's node list
	//}

	//for (int i = 0; i < numberOfGenes1; i++) { //run through all genes in parent 1
	//	geneHash.Add(geneList1[i].getInnovationNumber(), new NEATGene[]{ geneList1[i], null }); //add into the hash with innovation number as the key and gene array of size 2 as value
	//}

	//for (int i = 0; i < numberOfGenes2; i++) { //run through all genes in parent 2
	//	int innovationNumber = geneList2[i].getInnovationNumber(); //get innovation number 

	//	if (geneHash.ContainsKey(innovationNumber) == true) { //if there is a key in the hash with the given innovation number
	//		NEATGene[] geneValue = (NEATGene[])geneHash[innovationNumber]; //get gene array value with the innovation key
	//		geneValue[1] = geneList2[i]; //since this array already contains value in first location, we can add the new gene in the second location
	//		geneHash.Remove(innovationNumber); //remove old value with the key
	//		geneHash.Add(innovationNumber, geneValue); //add new value with the key
	//	}
	//	else { //there exists no key with the given innovation number
	//		geneHash.Add(innovationNumber, new NEATGene[]{ null , geneList2[i] }); //add into  the hash with innovation number as the key and gene array of size 2 as value
	//	}
	//}

	//ICollection keysCol = geneHash.Keys; //get all keys in the hash

	//NEATGene gene = null; //

	//int[] keys = new int[keysCol.Count]; //int array with size of nuumber of keys in the hash

	//keysCol.CopyTo(keys, 0); //copy Icollentions keys list to keys array
	//keys = keys.OrderBy(i = > i).ToArray(); //order keys in asending order

	//for (int i = 0; i < keys.Length; i++) { //run through all keys
	//	NEATGene[] geneValue = (NEATGene[])geneHash[keys[i]]; //get value at each index

	//														  //compare value is used to compare gene activation states in each parent 
	//	int compareValue = -1;
	//	//0 = both genes are true, 1 = both are false, 2 = one is false other is true
	//	//3 = gene is dominant in one of the parents and is true, 4 = gene is dominant in one of the parents and is false

	//	if (geneValue[0] != null && geneValue[1] != null) { //gene eixts in both parents
	//		int randomIndex = Random.Range(0, 2);

	//		if (geneValue[0].GetGeneState() == true && geneValue[1].GetGeneState() == true) { //gene is true in both
	//			compareValue = 0; //set compared value to 0
	//		}
	//		else if (geneValue[0].GetGeneState() == false && geneValue[1].GetGeneState() == false) { //gene is false in both
	//			compareValue = 1; //set compared value to 1
	//		}
	//		else { //gene is true in one and false in the other
	//			compareValue = 2; //set compared value to 2
	//		}

	//		gene = CrossoverCopyGene(geneValue[randomIndex], compareValue); //randomly pick a gene from eaither parent and create deep copy 
	//		childGeneList.Add(gene); //add gene to the child gene list
	//	}
	//	else if (parent1.GetNetFitness() > parent2.GetNetFitness()) { //parent 1's fitness is greater than parent 2
	//		if (geneValue[0] != null) { //gene value at first index from parent 1 exists
	//			if (geneValue[0].GetGeneState() == true) { //gene is active
	//				compareValue = 3; //set compared value to 3
	//			}
	//			else { //gene is not active
	//				compareValue = 4; //set compared value to 4
	//			}

	//			gene = CrossoverCopyGene(geneValue[0], compareValue); //deep copy parent 1's gene
	//			childGeneList.Add(gene); //add gene to the child gene list
	//		}
	//	}
	//	else if (parent1.GetNetFitness() < parent2.GetNetFitness()) { //parent 2's fitness is greater than parent 1
	//		if (geneValue[1] != null) { //gene value at second index from parent 2 exists
	//			if (geneValue[1].GetGeneState() == true) { //gene is active
	//				compareValue = 3; //set compared value to 3
	//			}
	//			else { //gene is not active
	//				compareValue = 4; //set compared value to 4
	//			}

	//			gene = CrossoverCopyGene(geneValue[1], compareValue); //deep copy parent 2's gene 
	//			childGeneList.Add(gene); //add gene to the child gene list
	//		}
	//	}
	//	else if (geneValue[0] != null) { //both parents have equal fitness and gene value at first index from parent 1 exists
	//		if (geneValue[0].GetGeneState() == true) { //gene is active
	//			compareValue = 3; //set compared value to 3
	//		}
	//		else { //gene is not active
	//			compareValue = 4; //set compared value to 4
	//		}

	//		gene = CrossoverCopyGene(geneValue[0], compareValue); //deep copy parent 1's gene 
	//		childGeneList.Add(gene); //add gene to the child gene list
	//	}
	//	else if (geneValue[1] != null) { //both parents have equal fitness and gene value at second index from parent 2 exists
	//		if (geneValue[1].GetGeneState() == true) { //gene is active
	//			compareValue = 3; //set compared value to 3
	//		}
	//		else { //gene is not active
	//			compareValue = 4; //set compared value to 4
	//		}

	//		gene = CrossoverCopyGene(geneValue[1], compareValue); //deep copy parent 2's gene 
	//		childGeneList.Add(gene); //add gene to the child gene list
	//	}
	//}

	//child = new NEATNet(consultor, numberOfInputs, numberOfOutputs, childNodeList, childGeneList); //create new child neural network 
	NEATNetwork child;
	return child; //return newly created neural network
}

NEATGene NEATNetwork::crossoverCopyGene(NEATGene gene, int compareValue)
{
	NEATGene copyGene(gene); //deep copy gene 

	int factor = 2;
	if (compareValue == 1) {
		int randomNumber = rand() % 25 * factor;
		if (randomNumber == 0) {
			copyGene.setGeneState(false);
		}
	}
	else if (compareValue == 2) {
		int randomNumber = rand() % 10 * factor;
		if (randomNumber == 0) {
			copyGene.setGeneState(true);
		}
	}
	else {
		int randomNumber = rand() % 25 * factor;
		if (randomNumber == 0) {
			copyGene.setGeneState(!copyGene.getGeneState());
		}
	}

	return copyGene; //return new gene
}


void NEATNetwork::printNetwork()
{
	for (int i = 0; i < m_nodes.size(); i++) {
		std::cout << m_nodes[i].getNeuronType() << " ";
	}
	std::cout << "\n";
}

NEATNetwork::~NEATNetwork()
{
}