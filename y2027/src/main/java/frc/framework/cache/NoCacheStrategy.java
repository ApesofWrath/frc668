package frc.framework.cache;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.NodeCachableResult;

/**
 * A caching strategy that never caches the outputs of a node
 */
public class NoCacheStrategy implements CacheStrategy {
	@Override
	public boolean isCacheValid(long time, ExecutionManager manager, NodeCachableResult previousResult) {
		return false;
	}
}
