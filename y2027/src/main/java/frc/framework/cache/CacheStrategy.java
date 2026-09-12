package frc.framework.cache;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.SystemCachableResult;

public interface CacheStrategy {
	boolean isCacheValid(
			long time,
			ExecutionManager manager,
			SystemCachableResult previousResult
	);
}
