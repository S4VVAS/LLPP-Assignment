#include "MainWindow.h"

#include <QGraphicsView>
#include <QtGui>
#include <QBrush>

#include <iostream>
#include "simd_funcs.h"

#include <stdlib.h>

MainWindow::MainWindow(const Ped::Model &pedModel) : model(pedModel)
{
	// The Window 
	graphicsView = new QGraphicsView();

	setCentralWidget(graphicsView);

	// A surface for managing a large number of 2D graphical items
	scene = new QGraphicsScene(QRect(0, 0, 800, 600), this);

	// Connect
	graphicsView->setScene(scene);

	// Paint on surface
	scene->setBackgroundBrush(Qt::white);

	for (int x = 0; x <= 800; x += cellsizePixel){
		scene->addLine(x, 0, x, 600, QPen(Qt::gray));
	}

	// Now add the horizontal lines, paint them gray
	for (int y = 0; y <= 600; y += cellsizePixel){
		scene->addLine(0, y, 800, y, QPen(Qt::gray));
	}

	// Adds red lines defining borders of regions
	for (int i = 0; i < pedModel.regions.size(); i++){
		int x1 = pedModel.regions[i]->x1 * cellsizePixel;
		int x2 = pedModel.regions[i]->x2 * cellsizePixel;
		int y1 = pedModel.regions[i]->y1 * cellsizePixel;
		int y2 = pedModel.regions[i]->y2 * cellsizePixel;

		scene->addLine(x1, y1, x2, y1, QPen(Qt::red));
		scene->addLine(x1, y2, x2, y2, QPen(Qt::red));
		scene->addLine(x1, y1, x1, y2, QPen(Qt::red));
		scene->addLine(x2, y1, x2, y2, QPen(Qt::red));
	}

	// Create viewAgents with references to the position of the model counterparts
	const std::vector<Ped::Tagent*> &agents = model.getAgents();

	std::vector<Ped::Tagent*>::const_iterator it;

	int agentN = 0;
	for (it = agents.begin(); it != agents.end(); it++)
	{
		viewAgents.push_back(new ViewAgent(*it, scene, agentN++));
	}

	const int heatmapSize = model.getHeatmapSize();
	QPixmap pixmapDummy = QPixmap(heatmapSize, heatmapSize);
	pixmap = scene->addPixmap(pixmapDummy);

	paint();
	graphicsView->show(); // Redundant? 
}

void MainWindow::paint() {

	// Uncomment this to paint the heatmap (Assignment 4)
	// const int heatmapSize = model.getHeatmapSize();
	// QImage image((uchar*)*model.getHeatmap(), heatmapSize, heatmapSize, heatmapSize * sizeof(int), QImage::Format_ARGB32);
	QImage image;
	 pixmap->setPixmap(QPixmap::fromImage(image));

	// Paint all agents: green, if the only agent on that position, otherwise red
	std::set<std::tuple<int, int> > positionsTaken;
	std::vector<ViewAgent*>::iterator it;
	
	for (it = viewAgents.begin(); it != viewAgents.end(); it++)
	{
		size_t tupleSizeBeforeInsert = positionsTaken.size();
		positionsTaken.insert((*it)->getPosition());
		size_t tupleSizeAfterInsert = positionsTaken.size();

		QColor color;
		if (tupleSizeBeforeInsert != tupleSizeAfterInsert) {
			color = Qt::green;
		}
		else {
			color = Qt::red;
		}

		//TODO: 
		//Asserts that no collisions (stacked agents) occur 
		assert(color == Qt::green);

		(*it)->paint(color, model.SIMD);
	}
}

int MainWindow::cellToPixel(int val)
{
	return val*cellsizePixel;
}
MainWindow::~MainWindow()
{
	for_each(viewAgents.begin(), viewAgents.end(), [](ViewAgent * agent){delete agent; });
}
