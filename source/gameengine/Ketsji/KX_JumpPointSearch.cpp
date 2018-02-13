/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor(s): Ulysse Martin.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file gameengine/Ketsji/KX_JumpPointSearch.cpp
*  \ingroup ketsji
*/

#include "KX_JumpPointSearch.h"
#include "KX_GameObject.h"

#include <math.h>

KX_JumpPointSearch::Node::Node()
{
}

KX_JumpPointSearch::Node::Node(int x, int y)
	:m_x(x),
	m_y(y),
	m_fCost(0),
	m_gCost(0),
	m_parent(nullptr),
	m_opened(false),
	m_closed(false),
	m_isWalkable(true)
{
}

KX_JumpPointSearch::Node::~Node()
{
}

KX_JumpPointSearch::KX_JumpPointSearch()
{
	InitGrid();
}

KX_JumpPointSearch::~KX_JumpPointSearch()
{
}

void KX_JumpPointSearch::InitGrid()
{
	for (int x = 0; x < MAX_RESOLUTION; x++) {
		for (int y = 0; y < MAX_RESOLUTION; y++) {
			m_grid[x][y] = Node(x, y);
		}
	}
	for (int x = 0; x < MAX_RESOLUTION; x++) {
		m_grid[0][x].m_isWalkable = false;
		m_grid[MAX_RESOLUTION - 1][x].m_isWalkable = false;
		m_grid[x][0].m_isWalkable = false;
		m_grid[x][MAX_RESOLUTION - 1].m_isWalkable = false;
	}
}

KX_JumpPointSearch::Node *KX_JumpPointSearch::GetNodeAt(int x, int y)
{
	return &m_grid[x][y];
}

float KX_JumpPointSearch::Distance(Node *node1, Node *node2)
{
	float a = node2->m_x - node1->m_x;
	float b = node2->m_y - node1->m_y;
	return (fabs(a) + fabs(b));
}

JPath *KX_JumpPointSearch::ExpandPath(JPath *path)
{
	JPath *expanded = {};
	int l = path->size();

	if (l < 2) {
		return expanded;
	}

	for (int i = 0; i < l - 1; i++) {
		std::pair<float, float> coord0 = path->at(i);
		std::pair<float, float> coord1 = path->at(i + 1);

		JPath *interpolated = Interpolate(coord0.first, coord0.second, coord1.first, coord1.second);
		int interpolatedLen = interpolated->size();
		for (int j = 0; j < interpolatedLen - 1; j++) {
			expanded->push_back(interpolated->at(j));
		}
	}
	expanded->push_back(path->at(l - 1));

	return expanded;
}

JPath *KX_JumpPointSearch::Interpolate(float x0, float y0, float x1, float y1)
{
	JPath *line = {};

	float dx = fabs(x1 - x0);
	float dy = fabs(y1 - y0);

	int sx, sy;

	if (x0 < x1) {
		sx = 1;
	}
	else {
		sx = -1;
	}
	if (y0 < y1) {
		sy = 1;
	}
	else {
		sy = -1;
	}

	float err = dx - dy;

	while (1) {
		line->push_back({ x0, y0 });

		if (x0 == x1 && y0 == y1) {
			break;
		}

		float e2 = 2.0f * err;

		if (e2 > -dy) {
			err = err - dy;
			x0 = x0 + sx;
		}
		if (e2 < dx) {
			err = err + dx;
			y0 = y0 + sy;
		}
	}
	return line;
}

JPath KX_JumpPointSearch::BackTrace(Node *node)
{
	JPath path;
	path.push_back({ node->m_x, node->m_y });
	while (node->m_parent) {
		node = node->m_parent;
		path.push_back({ node->m_x, node->m_y });
	}
	std::reverse(path.begin(), path.end());
	return path;
}

static bool f_cost_inferior(KX_JumpPointSearch::Node *a, KX_JumpPointSearch::Node *b)
{
	return (a->m_fCost < b->m_fCost);
}

std::vector<KX_JumpPointSearch::Node *>KX_JumpPointSearch::SortByFcost(std::vector<Node *>inputNodes)
{
	std::sort(inputNodes.begin(), inputNodes.end(), f_cost_inferior);
	return inputNodes;
}

JPath *KX_JumpPointSearch::FindPath(Node *startNode, Node *endNode)
{
	m_openList.clear();

	startNode->m_gCost = 0;
	startNode->m_fCost = 0;
	m_nodesToClean.push_back(endNode);

	m_openList.push_back(startNode);
	startNode->m_opened = true;

	while (m_openList.size()) {
		m_openList = SortByFcost(m_openList);

		Node *node = m_openList[-1];
		m_openList.pop_back();

		m_nodesToClean.push_back(node);
		node->m_closed = true;

		if (node == endNode) {
			JPath *exp = ExpandPath(&BackTrace(endNode));
			return exp;
		}
		IdentifySuccessors(node, endNode, m_openList);
	}
	std::cout << "path not found" << std::endl;
	return {};
}

int KX_JumpPointSearch::GetPathForPython(MT_Vector3& fromPoint, MT_Vector3& toPoint, float *path)
{
	float fx, fy;
	float tx, ty;

	fx = fromPoint.x();
	fy = fromPoint.y();
	tx = toPoint.x();
	ty = toPoint.y();

	Node *from = GetNodeAt(int(fx), int(fy));
	Node *to = GetNodeAt(int(tx), int(ty));

	JPath *p = FindPath(from, to);

	int pathLen = 0;
	for (int i = 0; i < p->size(); i++) {
		path[i] = p->at(i).first;
		path[i + 1] = p->at(i).second;
		path[i + 2] = toPoint.z();
		pathLen++;
	}
	return pathLen;
}

void KX_JumpPointSearch::IdentifySuccessors(Node *node, Node *endNode, std::vector<Node *>openList)
{
	int x = node->m_x;
	int y = node->m_y;

	std::vector<std::pair<int, int>>neighbors = FindNeighbors(node);
	for (int i = 0; i < neighbors.size(); i++) {
		std::pair<int, int>neighbor = neighbors[i];
		std::pair<int, int>jumpPoint = Jump(neighbor.first, neighbor.second, x, y, endNode);
		if (jumpPoint != std::pair<int, int>({})) {
			int jx = jumpPoint.first;
			int jy = jumpPoint.second;

			Node *jumpNode = GetNodeAt(jx, jy);

			if (jumpNode->m_closed) {
				continue;
			}
			float d = Distance(jumpNode, node);
			float ng = node->m_gCost + d;

			if (!(jumpNode->m_opened || ng < jumpNode->m_gCost)) {
				jumpNode->m_gCost = ng;
				jumpNode->m_hCost = jumpNode->m_hCost || Distance(jumpNode, endNode);
				jumpNode->m_fCost = jumpNode->m_gCost + jumpNode->m_hCost;
				jumpNode->m_parent = node;

				if (!(jumpNode->m_opened)) {
					m_openList.push_back(jumpNode);
					m_nodesToClean.push_back(jumpNode);
					jumpNode->m_opened = true;
				}
			}
		}
	}
}

std::pair<int, int>KX_JumpPointSearch::Jump(int x, int y, int px, int py, Node *endNode)
{
	int dx = x - px;
	int dy = y - py;

	if (!(m_grid[x][y].m_isWalkable)) {
		return {};
	}
	if (GetNodeAt(x, y) == endNode) {
		return { x, y };
	}
	if (dx != 0 && dy != 0) {
		if (m_grid[x - dx][y + dy].m_isWalkable && !(m_grid[x - dx][y].m_isWalkable) ||
			m_grid[x + dx][y - dy].m_isWalkable && !(m_grid[x][y - dy].m_isWalkable)) {
			return { x, y };
		}
	}
	else {
		if (dx != 0) {
			if (m_grid[x + dx][y + 1].m_isWalkable && !(m_grid[x][y + 1].m_isWalkable) ||
				m_grid[x + dx][y - 1].m_isWalkable && !(m_grid[x][y - 1].m_isWalkable)) {
				return { x, y };
			}
		}
		else {
			if (m_grid[x + 1][y + dy].m_isWalkable && !(m_grid[x + 1][y].m_isWalkable) ||
				m_grid[x - 1][y + dy].m_isWalkable && !m_grid[x - 1][y].m_isWalkable) {
				return { x, y };
			}
		}
	}
	if (dx != 0 && dy != 0) {
		if (Jump(x + dx, y, x, y, endNode) != std::pair<int, int>({}) ||
			Jump(x, y + dy, x, y, endNode) != std::pair<int, int>({})) {
			return { x, y };
		}
	}
	if (m_grid[x + dx][y].m_isWalkable || m_grid[x][y + dy].m_isWalkable) {
		return Jump(x + dx, y + dy, x, y, endNode);
	}
	else {
		return {};
	}
}

std::vector<std::pair<int, int>>KX_JumpPointSearch::FindNeighbors(Node *node)
{
	Node *parent = node->m_parent;
	int x = node->m_x;
	int y = node->m_y;

	std::vector<std::pair<int, int>> neighbors = {};

	if (parent) {
		int px = parent->m_x;
		int py = parent->m_y;

		int dx = round((x - px) / max_ff(fabs(x - px), 1));
		int dy = round((y - py) / max_ff(fabs(y - py), 1));

		if (dx != 0 && dy != 0) {
			if (m_grid[x][y + dy].m_isWalkable) {
				neighbors.push_back({ x, y + dy });
			}
			if (m_grid[x + dx][y].m_isWalkable) {
				neighbors.push_back({ x + dx, y });
			}
			if (m_grid[x][y + dy].m_isWalkable || m_grid[x + dx][y].m_isWalkable) {
				neighbors.push_back({ x + dx, y + dy });
			}
			if (!(m_grid[x - dx][y].m_isWalkable) && m_grid[x][y + dy].m_isWalkable) {
				neighbors.push_back({ x - dx, y - dy });
			}
			if (!(m_grid[x][y - dy].m_isWalkable) && m_grid[x + dx][y].m_isWalkable) {
				neighbors.push_back({ x + dx, y - dy });
			}
		}
		else {
			if (dx == 0) {
				if (m_grid[x][y + dy].m_isWalkable) {
					neighbors.push_back({ x, y + dy });
					if (!(m_grid[x + 1][y].m_isWalkable)) {
						neighbors.push_back({ x + 1, y + dy });
					}
					if (!(m_grid[x - 1][y].m_isWalkable)) {
						neighbors.push_back({ x - 1, y + dy });
					}
				}
			}
			else {
				if (m_grid[x + dx][y].m_isWalkable) {
					neighbors.push_back({ x + dx, y });
					if (!(m_grid[x][y + 1].m_isWalkable)) {
						neighbors.push_back({ x + dx, y + 1 });
					}
					if (!(m_grid[x][y - 1].m_isWalkable)) {
						neighbors.push_back({ x + dx, y - 1 });
					}
				}
			}
		}
	}
	else {
		std::vector<Node *>neighborNodes = GetNeighbors(node);
		for (Node *n : neighborNodes) {
			neighbors.push_back({ n->m_x, n->m_y });
		}
	}
	return neighbors;
}

std::vector<KX_JumpPointSearch::Node *>KX_JumpPointSearch::GetNeighbors(Node *node)
{
	int x = node->m_x;
	int y = node->m_y;

	std::vector<Node *> neighbors = {};

	if (m_grid[x][y - 1].m_isWalkable) {
		neighbors.push_back(GetNodeAt(x, y - 1));
	}
	if (m_grid[x + 1][y].m_isWalkable) {
		neighbors.push_back(GetNodeAt(x + 1, y));
	}
	if (m_grid[x][y + 1].m_isWalkable) {
		neighbors.push_back(GetNodeAt(x, y + 1));
	}
	if (m_grid[x - 1][y].m_isWalkable) {
		neighbors.push_back(GetNodeAt(x - 1, y));
	}
	if (m_grid[x - 1][y - 1].m_isWalkable) {
		neighbors.push_back(GetNodeAt(x - 1, y - 1));
	}
	if (m_grid[x + 1][y - 1].m_isWalkable) {
		neighbors.push_back(GetNodeAt(x + 1, y - 1));
	}
	if (m_grid[x + 1][y + 1].m_isWalkable) {
		neighbors.push_back(GetNodeAt(x + 1, y + 1));
	}
	if (m_grid[x - 1][y + 1].m_isWalkable) {
		neighbors.push_back(GetNodeAt(x - 1, y + 1));
	}
	return neighbors;
}
