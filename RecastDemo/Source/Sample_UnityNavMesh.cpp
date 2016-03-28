//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_UnityNavMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "RecastDump.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "NavMeshPruneTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "RecastAlloc.h"
#include <map>
#include <utility>

#ifdef WIN32
#	define snprintf _snprintf
#endif


Sample_UnityNavMesh::Sample_UnityNavMesh() :
	m_keepInterResults(true),
	m_totalBuildTimeMs(0),
	m_pmesh(0),
	m_dmesh(0),
	m_drawMode(DRAWMODE_NAVMESH)
{
	setTool(new NavMeshTesterTool);
}
		
Sample_UnityNavMesh::~Sample_UnityNavMesh()
{
	cleanup();
}
	
void Sample_UnityNavMesh::cleanup()
{
	rcFreePolyMesh(m_pmesh);
	m_pmesh = 0;
	rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = 0;
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
}
			
void Sample_UnityNavMesh::handleSettings()
{
	Sample::handleCommonSettings();
	
	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;

	imguiSeparator();
	
	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(msg);
	
	imguiSeparator();
}

void Sample_UnityNavMesh::handleTools()
{
	int type = !m_tool ? TOOL_NONE : m_tool->type();
	
	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Prune Navmesh", type == TOOL_NAVMESH_PRUNE))
	{
		setTool(new NavMeshPruneTool);
	}
	if (imguiCheck("Create Off-Mesh Connections", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (imguiCheck("Create Crowds", type == TOOL_CROWD))
	{
		setTool(new CrowdTool);
	}
	
	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
		m_tool->handleMenu();

	imguiUnindent();

}

void Sample_UnityNavMesh::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;

	if (m_geom)
	{
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_POLYMESH] = m_pmesh != 0;
		valid[DRAWMODE_POLYMESH_DETAIL] = m_dmesh != 0;
	}
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;

	if (unavail == MAX_DRAWMODE)
		return;

	imguiLabel("Draw");
	if (imguiCheck("Input Mesh", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_drawMode = DRAWMODE_MESH;
	if (imguiCheck("Navmesh", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_drawMode = DRAWMODE_NAVMESH;
	if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Navmesh Nodes", m_drawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_drawMode = DRAWMODE_NAVMESH_NODES;
	if (imguiCheck("Voxels", m_drawMode == DRAWMODE_VOXELS, valid[DRAWMODE_VOXELS]))
		m_drawMode = DRAWMODE_VOXELS;
	if (imguiCheck("Walkable Voxels", m_drawMode == DRAWMODE_VOXELS_WALKABLE, valid[DRAWMODE_VOXELS_WALKABLE]))
		m_drawMode = DRAWMODE_VOXELS_WALKABLE;
	if (imguiCheck("Compact", m_drawMode == DRAWMODE_COMPACT, valid[DRAWMODE_COMPACT]))
		m_drawMode = DRAWMODE_COMPACT;
	if (imguiCheck("Compact Distance", m_drawMode == DRAWMODE_COMPACT_DISTANCE, valid[DRAWMODE_COMPACT_DISTANCE]))
		m_drawMode = DRAWMODE_COMPACT_DISTANCE;
	if (imguiCheck("Compact Regions", m_drawMode == DRAWMODE_COMPACT_REGIONS, valid[DRAWMODE_COMPACT_REGIONS]))
		m_drawMode = DRAWMODE_COMPACT_REGIONS;
	if (imguiCheck("Region Connections", m_drawMode == DRAWMODE_REGION_CONNECTIONS, valid[DRAWMODE_REGION_CONNECTIONS]))
		m_drawMode = DRAWMODE_REGION_CONNECTIONS;
	if (imguiCheck("Raw Contours", m_drawMode == DRAWMODE_RAW_CONTOURS, valid[DRAWMODE_RAW_CONTOURS]))
		m_drawMode = DRAWMODE_RAW_CONTOURS;
	if (imguiCheck("Both Contours", m_drawMode == DRAWMODE_BOTH_CONTOURS, valid[DRAWMODE_BOTH_CONTOURS]))
		m_drawMode = DRAWMODE_BOTH_CONTOURS;
	if (imguiCheck("Contours", m_drawMode == DRAWMODE_CONTOURS, valid[DRAWMODE_CONTOURS]))
		m_drawMode = DRAWMODE_CONTOURS;
	if (imguiCheck("Poly Mesh", m_drawMode == DRAWMODE_POLYMESH, valid[DRAWMODE_POLYMESH]))
		m_drawMode = DRAWMODE_POLYMESH;
	if (imguiCheck("Poly Mesh Detail", m_drawMode == DRAWMODE_POLYMESH_DETAIL, valid[DRAWMODE_POLYMESH_DETAIL]))
		m_drawMode = DRAWMODE_POLYMESH_DETAIL;
		
	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("to see more debug mode options.");
	}
}

void Sample_UnityNavMesh::handleRender()
{
	if (!m_geom || !m_geom->getMesh())
		return;
	
	DebugDrawGL dd;
	
	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);

	const float texScale = 1.0f / (m_cellSize * 10.0f);
	
	if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
								m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(),
								m_agentMaxSlope, texScale);
		m_geom->drawOffMeshConnections(&dd);
	}
	
	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
	dd.begin(DU_DRAW_POINTS, 5.0f);
	dd.vertex(bmin[0],bmin[1],bmin[2],duRGBA(255,255,255,128));
	dd.end();
	
	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
		m_drawMode == DRAWMODE_NAVMESH_TRANS ||
		m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_drawMode == DRAWMODE_NAVMESH_NODES ||
		m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&dd, *m_navQuery);
		duDebugDrawNavMeshPolysWithFlags(&dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}
		
	glDepthMask(GL_TRUE);

	if (m_pmesh && m_drawMode == DRAWMODE_POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&dd, *m_pmesh);
		glDepthMask(GL_TRUE);
	}
	if (m_dmesh && m_drawMode == DRAWMODE_POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&dd, *m_dmesh);
		glDepthMask(GL_TRUE);
	}
	
	m_geom->drawConvexVolumes(&dd);

	if (m_tool)
		m_tool->handleRender();
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_UnityNavMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);
}

void Sample_UnityNavMesh::handleMeshChanged(class InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

struct Line
{
	unsigned short* a;
	unsigned short* b;

	unsigned int hash()
	{
		unsigned int ha = (a[0] * (unsigned int)13 + a[1]) * 13 + a[2];
		unsigned int hb = (b[0] * (unsigned int)13 + b[1]) * 13 + b[2];
		if (ha > hb) std::swap(ha, hb);
		return ha * 10009 + hb;
	}
};

bool Sample_UnityNavMesh::handleBuild()
{
	if (!m_geom || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}
	
	cleanup();
	
	// Reset build times gathering.
	m_ctx->resetTimers();

	// Start the build process.	
	m_ctx->startTimer(RC_TIMER_TOTAL);

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	const float* verts = m_geom->getMesh()->getVerts();
	const int nverts = m_geom->getMesh()->getVertCount();
	const int* tris = m_geom->getMesh()->getTris();
	const int ntris = m_geom->getMesh()->getTriCount();
	
	m_pmesh = rcAllocPolyMesh();
	m_pmesh->cs = m_cellSize;
	m_pmesh->ch = m_cellHeight;
	m_pmesh->borderSize = 0.f;
	m_pmesh->maxEdgeError = m_edgeMaxError;
	rcVcopy(m_pmesh->bmin, bmin);
	rcVcopy(m_pmesh->bmax, bmax);
	m_pmesh->nverts = nverts;
	m_pmesh->verts = (unsigned short*)rcAlloc(sizeof(unsigned short) * nverts * 3, RC_ALLOC_PERM);
	float ics = 1.0f / m_cellSize;
	float ich = 1.0f / m_cellHeight;
	for (int i = 0; i < nverts; ++i)
	{
		m_pmesh->verts[i * 3 + 0] = (unsigned short)((verts[i * 3 + 0] - bmin[0]) * ics);
		m_pmesh->verts[i * 3 + 1] = (unsigned short)((verts[i * 3 + 1] - bmin[1])* ich);
		m_pmesh->verts[i * 3 + 2] = (unsigned short)((verts[i * 3 + 2] - bmin[2]) * ics);
	}

	for (int i = 0; i < nverts; ++i)
	{
		m_ctx->log(RC_LOG_PROGRESS, "vert %d %d %d",
			m_pmesh->verts[i * 3 + 0],
			m_pmesh->verts[i * 3 + 1],
			m_pmesh->verts[i * 3 + 2]);
	}

	m_pmesh->nvp = 3;
	m_pmesh->npolys = ntris;
	m_pmesh->polys = (unsigned short*)rcAlloc(sizeof(unsigned short) * ntris * 3 * 2, RC_ALLOC_PERM);
	std::map<unsigned int, unsigned short> edgeHash;
	for (int i = 0; i < ntris; ++i)
	{
		m_pmesh->polys[i * 6 + 0] = tris[i * 3 + 0];
		m_pmesh->polys[i * 6 + 1] = tris[i * 3 + 1];
		m_pmesh->polys[i * 6 + 2] = tris[i * 3 + 2];
		m_pmesh->polys[i * 6 + 3] = -1;
		m_pmesh->polys[i * 6 + 4] = -1;
		m_pmesh->polys[i * 6 + 5] = -1;

		Line line;
		unsigned int h;
		line.a = &(m_pmesh->verts[m_pmesh->polys[i * 6 + 0] * 3]);
		line.b = &(m_pmesh->verts[m_pmesh->polys[i * 6 + 1] * 3]);
		h = line.hash();
		if (edgeHash.find(h) == edgeHash.end())
		{
			edgeHash[h] = i * 3 + 0;
		}
		else
		{
			unsigned n = edgeHash[h];
			m_pmesh->polys[(n / 3) * 6 + 3 + (n % 3)] = i;
			m_pmesh->polys[i * 6 + 3 + 0] = (n / 3);
			edgeHash.erase(h);
		}
		
		line.a = &(m_pmesh->verts[m_pmesh->polys[i * 6 + 1] * 3]);
		line.b = &(m_pmesh->verts[m_pmesh->polys[i * 6 + 2] * 3]);
		h = line.hash();
		if (edgeHash.find(h) == edgeHash.end())
		{
			edgeHash[h] = i * 3 + 1;
		}
		else
		{
			unsigned n = edgeHash[h];
			m_pmesh->polys[(n / 3) * 6 + 3 + (n % 3)] = i;
			m_pmesh->polys[i * 6 + 3 + 1] = (n / 3);
			edgeHash.erase(h);
		}

		line.a = &(m_pmesh->verts[m_pmesh->polys[i * 6 + 2] * 3]);
		line.b = &(m_pmesh->verts[m_pmesh->polys[i * 6 + 0] * 3]);
		h = line.hash();
		if (edgeHash.find(h) == edgeHash.end())
		{
			edgeHash[h] = i * 3 + 2;
		}
		else
		{
			unsigned n = edgeHash[h];
			m_pmesh->polys[(n / 3) * 6 + 3 + (n % 3)] = i;
			m_pmesh->polys[i * 6 + 3 + 2] = (n / 3);
			edgeHash.erase(h);
		}
	}

	for (int i = 0; i < ntris; ++i)
	{
		m_ctx->log(RC_LOG_PROGRESS, "poly %d %d %d, %d %d %d",
			m_pmesh->polys[i * 6 + 0],
			m_pmesh->polys[i * 6 + 1],
			m_pmesh->polys[i * 6 + 2],
			m_pmesh->polys[i * 6 + 3],
			m_pmesh->polys[i * 6 + 4],
			m_pmesh->polys[i * 6 + 5]);
	}

	m_pmesh->regs = (unsigned short*)rcAlloc(sizeof(unsigned short) * ntris, RC_ALLOC_PERM);
	memset(m_pmesh->regs, 0, sizeof(unsigned short)*ntris);
	m_pmesh->areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * ntris, RC_ALLOC_PERM);
	memset(m_pmesh->areas, 0, sizeof(unsigned char)*ntris);
	m_pmesh->flags = (unsigned short*)rcAlloc(sizeof(unsigned short) * ntris, RC_ALLOC_PERM);
	memset(m_pmesh->flags, 0, sizeof(unsigned short) * ntris);
	for (int i = 0; i < ntris; ++i)
	{
		m_pmesh->flags[i] |= 0x1;
	}
	// p_mesh

	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	m_dmesh->nmeshes = 0;
	m_dmesh->nverts = 0;
	m_dmesh->ntris = 0;
	// d_mesh

	unsigned char* navData = 0;
	int navDataSize = 0;

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = m_pmesh->verts;
	params.vertCount = m_pmesh->nverts;
	params.polys = m_pmesh->polys;
	params.polyAreas = m_pmesh->areas;
	params.polyFlags = m_pmesh->flags;
	params.polyCount = m_pmesh->npolys;
	params.nvp = m_pmesh->nvp;
	params.detailMeshes = m_dmesh->meshes;
	params.detailVerts = m_dmesh->verts;
	params.detailVertsCount = m_dmesh->nverts;
	params.detailTris = m_dmesh->tris;
	params.detailTriCount = m_dmesh->ntris;
	params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
	params.offMeshConRad = m_geom->getOffMeshConnectionRads();
	params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
	params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
	params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
	params.offMeshConUserID = m_geom->getOffMeshConnectionId();
	params.offMeshConCount = m_geom->getOffMeshConnectionCount();
	params.walkableHeight = m_agentHeight;
	params.walkableRadius = m_agentRadius;
	params.walkableClimb = m_agentMaxClimb;
	rcVcopy(params.bmin, m_pmesh->bmin);
	rcVcopy(params.bmax, m_pmesh->bmax);
	params.cs = m_pmesh->cs;
	params.ch = m_pmesh->ch;
	params.buildBvTree = true;

	if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
	{
		m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
		return false;
	}

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		dtFree(navData);
		m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
		return false;
	}

	dtStatus status;

	status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		dtFree(navData);
		m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
		return false;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
		return false;
	}
	
	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);
	
	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
	
	if (m_tool)
		m_tool->init(this);
	initToolStates(this);

	return true;
}
