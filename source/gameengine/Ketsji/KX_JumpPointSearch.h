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

/** \file KX_JumpPointSearch.h
*  \ingroup ketsji
*/

#ifndef __KX_JUMPPOINTSEARCH_H__
#define __KX_JUMPPOINTSEARCH_H__

#include <vector>

#define MAX_RESOLUTION 256

typedef std::vector<std::pair<float, float>> JPath;

class MT_Vector3;

class KX_JumpPointSearch
{

public:
	KX_JumpPointSearch();
	virtual ~KX_JumpPointSearch();

	class Node {
	public:
		Node();
		Node(int x, int y);
		virtual ~Node();
		class Node *m_parent;
		int m_x, m_y;
		float m_fCost, m_gCost, m_hCost;
		bool m_opened, m_closed, m_isWalkable;
	};

	Node m_grid[MAX_RESOLUTION][MAX_RESOLUTION];

	std::vector<Node *>m_nodesToClean;
	std::vector<Node *>m_openList;

	void InitGrid();
	Node *GetNodeAt(int x, int y);
	float Distance(Node *n1, Node *n2);
	JPath ExpandPath(JPath *inputPath);
	JPath Interpolate(float x0, float y0, float x1, float y1);
	JPath BackTrace(Node *node);
	JPath FindPath(Node *startNode, Node *endNode);
	std::vector<Node *>SortByFcost(std::vector<Node *>inputNodes);
	void IdentifySuccessors(Node *node, Node *endNode, std::vector<Node *>openList);
	std::pair<int, int>Jump(int x, int y, int px, int py, Node *endNode);
	std::vector<std::pair<int, int>>FindNeighbors(Node *node);
	std::vector<Node *>GetNeighbors(Node *node);
	int GetPathForPython(MT_Vector3& fromPoint, MT_Vector3& toPoint, float *path);



};

#endif  // __KX_JUMPPOINTSEARCH_H__
